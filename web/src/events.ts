import { ref, onMounted, onUnmounted } from 'vue';
import useApi from './api';
import JobrunnerStatus from './types/JobrunnerStatus';

// The latest jobrunner status from the event stream.
export const jobrunnerStatus = ref<JobrunnerStatus | null>( null );

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
 * Keeps the event stream open while the calling component is mounted.  All components
 * in the page share one stream.
 */
export default function useEvents() {
	onMounted( subscribe );
	onUnmounted( unsubscribe );

	return { jobrunnerStatus };
}
