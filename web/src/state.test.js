import { beforeEach, describe, expect, it, vi } from 'vitest';
import { createPinia, setActivePinia } from 'pinia';
import { notificationsStore } from './state';

describe( 'notificationsStore notifyJobFinished', () => {
	beforeEach( () => {
		setActivePinia( createPinia() );
		const notificationMock = vi.fn();
		notificationMock.permission = 'granted';
		notificationMock.requestPermission = vi.fn();
		global.Notification = notificationMock;

		// jsdom does not implement the Web Locks API; stub it with a
		// pass-through that runs the callback immediately.
		Object.defineProperty( global.navigator, 'locks', {
			configurable: true,
			value: { request: ( _name, cb ) => Promise.resolve( cb() ) }
		} );

		localStorage.clear();
	} );

	it( 'notifies once when tracked job finishes successfully', async () => {
		localStorage.setItem( 'spiderpig-job-id', JSON.stringify( 22 ) );
		const store = notificationsStore();

		await store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished successfully'
		} );
		expect( store.jobId ).toBeNull();
	} );

	it( 'does not notify when finished job is not tracked by this user', async () => {
		localStorage.setItem( 'spiderpig-job-id', JSON.stringify( 22 ) );
		const store = notificationsStore();

		await store.notifyJobFinished( {
			id: 23,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).not.toHaveBeenCalled();
		expect( store.jobId ).toBe( JSON.stringify( 22 ) );
	} );

	it( 'uses error text when tracked job finishes with a non-zero exit status', async () => {
		localStorage.setItem( 'spiderpig-job-id', JSON.stringify( 22 ) );
		const store = notificationsStore();

		await store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 1
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished with errors'
		} );
		expect( store.jobId ).toBeNull();
	} );

	it( 'does not double-notify if the job id was cleared by a sibling tab while waiting for the lock', async () => {
		localStorage.setItem( 'spiderpig-job-id', JSON.stringify( 22 ) );
		const store = notificationsStore();

		Object.defineProperty( global.navigator, 'locks', {
			configurable: true,
			value: {
				request: ( _name, cb ) => {
					localStorage.removeItem( 'spiderpig-job-id' );
					return Promise.resolve( cb() );
				}
			}
		} );

		await store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).not.toHaveBeenCalled();
	} );
} );
