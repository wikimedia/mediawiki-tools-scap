import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { flushPromises, mount } from '@vue/test-utils';
import { defineComponent, h } from 'vue';
import useEvents, { jobrunnerStatus } from './events';

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

describe( 'event stream', () => {
	beforeEach( () => {
		getEventsMock.mockReset();
		jobrunnerStatus.value = null;
	} );

	it( 'sets the jobrunner status from each jobrunner event', async () => {
		getEventsMock.mockImplementation( async ( signal ) => streamResponse( [
			'{"type": "jobrunner", "data": {"status": "idle"}}\n{"type": "jo',
			'bs_digest", "data": "a"}\n{"type": "keepalive"}\n',
			'{"type": "jobrunner", "data": {"status": "running"}}\n'
		], signal ) );

		const wrapper = mountWithEvents();
		await vi.waitFor( () => expect( jobrunnerStatus.value ).toEqual( { status: 'running' } ) );

		expect( getEventsMock ).toHaveBeenCalledTimes( 1 );
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
	} );
} );
