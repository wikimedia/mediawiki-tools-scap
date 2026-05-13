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
	} );

	it( 'notifies once when tracked job finishes successfully', () => {
		const store = notificationsStore();
		store.jobId = JSON.stringify( 22 );

		store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished successfully'
		} );
		expect( store.jobId ).toBeNull();
	} );

	it( 'does not notify when finished job is not tracked by this user', () => {
		const store = notificationsStore();
		store.jobId = JSON.stringify( 22 );

		store.notifyJobFinished( {
			id: 23,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).not.toHaveBeenCalled();
		expect( store.jobId ).toBe( JSON.stringify( 22 ) );
	} );

	it( 'uses error text when tracked job finishes with a non-zero exit status', () => {
		const store = notificationsStore();
		store.jobId = JSON.stringify( 22 );

		store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 1
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished with errors'
		} );
		expect( store.jobId ).toBeNull();
	} );
} );
