import { ref, watch, onMounted, onUnmounted, WatchSource } from 'vue';
import useApi from './api';
import JobrunnerStatus from './types/JobrunnerStatus';

// The latest jobrunner status from the event stream.
export const jobrunnerStatus = ref<JobrunnerStatus | null>( null );

// The digest of the job list.  See Job.digest() in scap/spiderpig/model.py.
export const jobsDigest = ref<string | null>( null );

// The digest of each unfinished job, keyed by the id of the job.  See
// Job.unfinished_digests() in scap/spiderpig/model.py.
export const unfinishedJobDigests = ref<Record<string, string>>( {} );

// The number of MediaWiki error records in the last logstash poll, or null while the
// server cannot read the poll results.
export const errorTotal = ref<number | null>( null );

// The digest of the error logs: the logstash poll results and the error notes.  See
// get_errors_digest() in scap/spiderpig/api.py.  Null while the server cannot read the
// poll results.
export const errorsDigest = ref<string | null>( null );

// The time (in seconds since the epoch) of the last successful logstash poll when that
// poll is too old, or null.
export const staleErrorPollTime = ref<number | null>( null );

let subscribers = 0;
let abortController: AbortController | null = null;

async function* records( reader: ReadableStreamDefaultReader<Uint8Array> ) {
	const decoder = new TextDecoder();
	let buffer = '';

	while ( true ) {
		const { done, value } = await reader.read();
		if ( done ) {
			return;
		}

		buffer += decoder.decode( value, { stream: true } );
		let newlineIndex: number;
		while ( ( newlineIndex = buffer.indexOf( '\n' ) ) !== -1 ) {
			const line = buffer.slice( 0, newlineIndex );
			buffer = buffer.slice( newlineIndex + 1 );
			yield JSON.parse( line );
		}
	}
}

function sleep( ms: number, signal: AbortSignal ): Promise<void> {
	return new Promise( ( resolve ) => {
		const timer = setTimeout( resolve, ms );
		signal.addEventListener( 'abort', () => {
			clearTimeout( timer );
			resolve();
		}, { once: true } );
	} );
}

// Reads the event stream until abort is signaled.  Reconnects after an error,
// after the server closes the stream, or after too much silence.
async function readEvents( signal: AbortSignal ) {
	const minRetryDelay = 1000;
	const maxRetryDelay = 30000;
	// The server sends a keepalive every 15 seconds.
	const silenceTimeout = 45000;
	let retryDelay = minRetryDelay;

	while ( !signal.aborted ) {
		const connectionController = new AbortController();
		const abortConnection = () => connectionController.abort();
		signal.addEventListener( 'abort', abortConnection, { once: true } );
		let silenceTimer: ReturnType<typeof setTimeout> | undefined;
		const restartSilenceTimer = () => {
			clearTimeout( silenceTimer );
			silenceTimer = setTimeout( () => connectionController.abort(
				new Error( `nothing received for ${ silenceTimeout / 1000 } seconds` )
			), silenceTimeout );
		};

		try {
			restartSilenceTimer();
			const response = await useApi().getEvents( connectionController.signal );
			if ( !response.body ) {
				throw new Error( 'the response has no body' );
			}
			for await ( const rec of records( response.body.getReader() ) ) {
				restartSilenceTimer();
				retryDelay = minRetryDelay;
				if ( rec.type === 'jobrunner' ) {
					jobrunnerStatus.value = rec.data;
				} else if ( rec.type === 'jobs_digest' ) {
					jobsDigest.value = rec.data;
				} else if ( rec.type === 'unfinished_job_digests' ) {
					unfinishedJobDigests.value = rec.data;
				} else if ( rec.type === 'error_total' ) {
					errorTotal.value = rec.data;
				} else if ( rec.type === 'errors_digest' ) {
					errorsDigest.value = rec.data;
				} else if ( rec.type === 'stale_error_poll_time' ) {
					staleErrorPollTime.value = rec.data;
				}
			}
		} catch ( error ) {
			if ( signal.aborted ) {
				return;
			}
			console.log( `Event stream ended (${ ( error as Error ).message }).  Reconnecting` );
		} finally {
			clearTimeout( silenceTimer );
			signal.removeEventListener( 'abort', abortConnection );
		}

		await sleep( retryDelay, signal );
		retryDelay = Math.min( retryDelay * 2, maxRetryDelay );
	}
}

function subscribe() {
	subscribers++;
	if ( subscribers === 1 ) {
		abortController = new AbortController();
		readEvents( abortController.signal );
	}
}

function unsubscribe() {
	subscribers--;
	if ( subscribers === 0 ) {
		abortController?.abort();
		abortController = null;
	}
}

/**
 * Returns the refs that the event stream updates, and arranges for the stream to stay
 * open while the calling component is mounted.  All components in the page share one
 * stream.
 */
export default function useEvents() {
	onMounted( subscribe );
	onUnmounted( unsubscribe );

	return {
		jobrunnerStatus,
		jobsDigest,
		unfinishedJobDigests,
		errorTotal,
		errorsDigest,
		staleErrorPollTime
	};
}

/**
 * Returns a function that calls updateFunc, and arranges for that function to be
 * called when the calling component mounts and each time the value of one of the
 * sources changes.
 *
 * Only one call to updateFunc is in progress at a time.  If the value of a source
 * changes or the returned function is called during a call, updateFunc is called once
 * more after it.
 */
function useUpdates( sources: WatchSource<unknown>[], updateFunc: () => Promise<void> ) {
	let updating = false;
	let pending = false;

	async function update() {
		if ( updating ) {
			pending = true;
			return;
		}

		updating = true;
		try {
			do {
				pending = false;
				try {
					await updateFunc();
				} catch ( error ) {
					console.error( ( error as Error ).message );
				}
			} while ( pending );
		} finally {
			updating = false;
		}
	}

	useEvents();
	onMounted( update );
	watch( sources, update );

	return update;
}

// See useUpdates().  Here updateFunc is called when the calling component mounts and
// with each change to the job list.
export function useJobListUpdates( updateFunc: () => Promise<void> ) {
	return useUpdates( [ jobsDigest ], updateFunc );
}

// See useUpdates().  Here updateFunc is called when the calling component mounts, when
// jobIdFunc() returns a different id, and with each change to that job.  When the job
// finishes, its entry leaves unfinishedJobDigests.  After the call for that change,
// updateFunc is not called again for that job.
export function useJobUpdates(
	jobIdFunc: () => string, updateFunc: () => Promise<void>
) {
	return useUpdates(
		[ jobIdFunc, () => unfinishedJobDigests.value[ jobIdFunc() ] ],
		updateFunc
	);
}

// See useUpdates().  Here updateFunc is called when the calling component mounts and
// with each change to the error logs.
export function useErrorLogsUpdates( updateFunc: () => Promise<void> ) {
	return useUpdates( [ errorsDigest ], updateFunc );
}
