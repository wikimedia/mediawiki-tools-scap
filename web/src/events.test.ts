import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { flushPromises, mount } from '@vue/test-utils';
import { defineComponent, h, ref } from 'vue';
import useEvents, {
	errorsDigest, errorTotal, jobrunnerStatus, jobsDigest, staleErrorPollTime,
	unfinishedJobDigests, useErrorLogsUpdates, useJobListUpdates, useJobUpdates
} from './events';

const getEventsMock = vi.fn();

vi.mock( './api', () => ( {
	default: () => ( {
		getEvents: getEventsMock
	} )
} ) );

// A response whose body sends each chunk, then stays open until the signal aborts.
function streamResponse( chunks: string[], signal: AbortSignal ) {
	const encoder = new TextEncoder();
	const body = new ReadableStream( {
		start( controller ) {
			for ( const chunk of chunks ) {
				controller.enqueue( encoder.encode( chunk ) );
			}
			signal.addEventListener( 'abort', () => controller.error( signal.reason ) );
		}
	} );
	return new Response( body );
}

function mountWithEvents() {
	return mount( defineComponent( {
		setup() {
			useEvents();
			return () => h( 'div' );
		}
	} ) );
}

function mountWithJobUpdates(
	jobIdFunc: () => string, updateFunc: () => Promise<void>
) {
	return mount( defineComponent( {
		setup() {
			useJobUpdates( jobIdFunc, updateFunc );
			return () => h( 'div' );
		}
	} ) );
}

function mountWithErrorLogsUpdates( updateFunc: () => Promise<void> ) {
	return mount( defineComponent( {
		setup() {
			useErrorLogsUpdates( updateFunc );
			return () => h( 'div' );
		}
	} ) );
}

function mountWithJobListUpdates( updateFunc: () => Promise<void> ) {
	let update!: () => Promise<void>;
	const wrapper = mount( defineComponent( {
		setup() {
			update = useJobListUpdates( updateFunc );
			return () => h( 'div' );
		}
	} ) );
	return { wrapper, update };
}

describe( 'event stream', () => {
	beforeEach( () => {
		getEventsMock.mockReset();
		jobrunnerStatus.value = null;
		jobsDigest.value = null;
		unfinishedJobDigests.value = {};
		errorTotal.value = null;
		errorsDigest.value = null;
		staleErrorPollTime.value = null;
	} );

	it( 'sets the jobrunner status and the job digests', async () => {
		getEventsMock.mockImplementation( async ( signal ) => streamResponse( [
			'{"type": "jobrunner", "data": {"status": "idle"}}\n{"type": "jo',
			'bs_digest", "data": "a"}\n{"type": "keepalive"}\n',
			'{"type": "jobrunner", "data": {"status": "running"}}\n',
			'{"type": "unfinished_job_digests", "data": {"42": "x"}}\n',
			'{"type": "jobs_digest", "data": "b"}\n'
		], signal ) );

		const wrapper = mountWithEvents();
		await vi.waitFor( () => expect( jobsDigest.value ).toBe( 'b' ) );

		expect( jobrunnerStatus.value ).toEqual( { status: 'running' } );
		expect( unfinishedJobDigests.value ).toEqual( { 42: 'x' } );
		expect( getEventsMock ).toHaveBeenCalledTimes( 1 );
		wrapper.unmount();
	} );

	it( 'sets the error total, the stale poll time and the error logs digest', async () => {
		getEventsMock.mockImplementation( async ( signal ) => streamResponse( [
			'{"type": "error_total", "data": 7}\n{"type": "errors_digest", "data": "a"}\n',
			'{"type": "stale_error_poll_time", "data": 1700000000.5}\n',
			'{"type": "error_total", "data": null}\n{"type": "errors_digest", "data": "b"}\n'
		], signal ) );

		const wrapper = mountWithEvents();
		await vi.waitFor( () => expect( errorsDigest.value ).toBe( 'b' ) );

		expect( errorTotal.value ).toBe( null );
		expect( staleErrorPollTime.value ).toBe( 1700000000.5 );
		wrapper.unmount();
	} );

	it( 'closes the stream when the last component unmounts', async () => {
		let streamSignal!: AbortSignal;
		getEventsMock.mockImplementation( async ( signal ) => {
			streamSignal = signal;
			return streamResponse( [], signal );
		} );

		const first = mountWithEvents();
		const second = mountWithEvents();
		await flushPromises();
		first.unmount();

		expect( getEventsMock ).toHaveBeenCalledTimes( 1 );
		expect( streamSignal.aborted ).toBe( false );

		second.unmount();

		expect( streamSignal.aborted ).toBe( true );
	} );

	describe( 'after the stream ends', () => {
		beforeEach( () => {
			vi.useFakeTimers();
		} );

		afterEach( () => {
			vi.useRealTimers();
		} );

		it( 'connects again', async () => {
			getEventsMock.mockRejectedValueOnce( new Error( 'HTTP request failed with status: 502' ) );
			getEventsMock.mockImplementation( async ( signal ) => streamResponse(
				[ '{"type": "jobrunner", "data": {"status": "idle"}}\n' ], signal
			) );
			vi.spyOn( console, 'log' ).mockImplementation( () => {} );

			const wrapper = mountWithEvents();
			await flushPromises();
			expect( jobrunnerStatus.value ).toBe( null );

			await vi.advanceTimersByTimeAsync( 1000 );

			expect( getEventsMock ).toHaveBeenCalledTimes( 2 );
			expect( jobrunnerStatus.value ).toEqual( { status: 'idle' } );
			wrapper.unmount();
		} );

		it( 'connects again when the stream goes silent', async () => {
			getEventsMock.mockImplementation( async ( signal ) => streamResponse(
				[ '{"type": "jobrunner", "data": {"status": "idle"}}\n' ], signal
			) );
			vi.spyOn( console, 'log' ).mockImplementation( () => {} );

			const wrapper = mountWithEvents();
			await vi.advanceTimersByTimeAsync( 44000 );
			expect( getEventsMock ).toHaveBeenCalledTimes( 1 );

			await vi.advanceTimersByTimeAsync( 2000 );

			expect( getEventsMock ).toHaveBeenCalledTimes( 2 );
			expect( console.log ).toHaveBeenCalledWith(
				'Event stream ended (nothing received for 45 seconds).  Reconnecting'
			);
			wrapper.unmount();
		} );

		it( 'does not update after a reconnect that brings no changes', async () => {
			const digests = '{"type": "jobs_digest", "data": "a"}\n{"type": "errors_digest", "data": "e"}\n';
			getEventsMock.mockResolvedValueOnce( new Response( digests ) );
			getEventsMock.mockImplementation( async ( signal ) => streamResponse( [ digests ], signal ) );
			vi.spyOn( console, 'log' ).mockImplementation( () => {} );
			const updateJobList = vi.fn( async () => {} );
			const updateErrorLogs = vi.fn( async () => {} );

			const { wrapper } = mountWithJobListUpdates( updateJobList );
			const errorLogsWrapper = mountWithErrorLogsUpdates( updateErrorLogs );
			await vi.advanceTimersByTimeAsync( 1000 );

			expect( getEventsMock ).toHaveBeenCalledTimes( 2 );
			// One call on mount and one for the first digest.
			expect( updateJobList ).toHaveBeenCalledTimes( 2 );
			expect( updateErrorLogs ).toHaveBeenCalledTimes( 2 );
			wrapper.unmount();
			errorLogsWrapper.unmount();
		} );
	} );
} );

describe( 'useJobListUpdates', () => {
	beforeEach( () => {
		getEventsMock.mockReset();
		getEventsMock.mockReturnValue( new Promise( () => {} ) );
	} );

	it( 'updates on mount and on each change to the job list', async () => {
		const updateFunc = vi.fn( async () => {} );

		const { wrapper } = mountWithJobListUpdates( updateFunc );
		await flushPromises();
		jobsDigest.value = 'changed';
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 2 );
		wrapper.unmount();
	} );

	it( 'calls updateFunc once more after the requests that come during a call', async () => {
		let finishUpdate!: () => void;
		const updateFunc = vi.fn( () => new Promise<void>( ( resolve ) => {
			finishUpdate = resolve;
		} ) );

		const { wrapper, update } = mountWithJobListUpdates( updateFunc );
		await flushPromises();
		update();
		update();
		jobsDigest.value = 'changed again';
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 1 );

		finishUpdate();
		await flushPromises();
		finishUpdate();
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 2 );
		wrapper.unmount();
	} );
} );

describe( 'useJobUpdates', () => {
	beforeEach( () => {
		getEventsMock.mockReset();
		getEventsMock.mockReturnValue( new Promise( () => {} ) );
		unfinishedJobDigests.value = { 42: 'a', 43: 'a' };
	} );

	it( 'updates on mount and on each change to the job, including its end', async () => {
		const updateFunc = vi.fn( async () => {} );

		const wrapper = mountWithJobUpdates( () => '42', updateFunc );
		await flushPromises();
		unfinishedJobDigests.value = { 42: 'b', 43: 'a' };
		await flushPromises();
		unfinishedJobDigests.value = { 43: 'a' };
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 3 );
		wrapper.unmount();
	} );

	it( 'ignores changes to other jobs', async () => {
		const updateFunc = vi.fn( async () => {} );

		const wrapper = mountWithJobUpdates( () => '42', updateFunc );
		await flushPromises();
		unfinishedJobDigests.value = { 42: 'a', 43: 'b' };
		await flushPromises();
		unfinishedJobDigests.value = { 42: 'a' };
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 1 );
		wrapper.unmount();
	} );

	it( 'updates once when the job id changes', async () => {
		const updateFunc = vi.fn( async () => {} );
		const jobId = ref( '42' );

		const wrapper = mountWithJobUpdates( () => jobId.value, updateFunc );
		await flushPromises();
		jobId.value = '44';
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 2 );
		wrapper.unmount();
	} );
} );

describe( 'useErrorLogsUpdates', () => {
	beforeEach( () => {
		getEventsMock.mockReset();
		getEventsMock.mockReturnValue( new Promise( () => {} ) );
	} );

	it( 'updates on mount and on each change to the error logs', async () => {
		const updateFunc = vi.fn( async () => {} );

		const wrapper = mountWithErrorLogsUpdates( updateFunc );
		await flushPromises();
		errorsDigest.value = 'changed';
		await flushPromises();

		expect( updateFunc ).toHaveBeenCalledTimes( 2 );
		wrapper.unmount();
	} );
} );
