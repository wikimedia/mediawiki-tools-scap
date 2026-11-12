import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { createPinia, setActivePinia } from 'pinia';
import useApi from './api';

describe( 'getEvents', () => {
	beforeEach( () => {
		setActivePinia( createPinia() );
		vi.stubGlobal( 'fetch', vi.fn( async () => new Response(
			JSON.stringify( { code: 'need2fa', message: 'Second factor required' } ),
			{ status: 401 }
		) ) );
		vi.spyOn( console, 'log' ).mockImplementation( () => {} );
	} );

	afterEach( () => {
		vi.unstubAllGlobals();
		window.history.pushState( {}, '', '/' );
	} );

	it( 'redirects to the login page after a 401', async () => {
		const api = useApi();
		const setWindowLocation = vi.spyOn( api, 'setWindowLocation' ).mockImplementation( () => {} );
		window.history.pushState( {}, '', '/mediawiki/logs' );

		await expect( api.getEvents( new AbortController().signal ) ).rejects.toThrow( 'Auth failed' );

		expect( setWindowLocation ).toHaveBeenCalledWith(
			expect.stringMatching( /^\/login\?redirect=/ )
		);
	} );

	it( 'does not redirect after a 401 on the login page', async () => {
		const api = useApi();
		const setWindowLocation = vi.spyOn( api, 'setWindowLocation' ).mockImplementation( () => {} );
		window.history.pushState( {}, '', '/login' );

		await expect( api.getEvents( new AbortController().signal ) ).rejects.toThrow( 'Auth failed' );

		expect( setWindowLocation ).not.toHaveBeenCalled();
	} );
} );
