import { beforeEach, describe, expect, it, vi } from 'vitest';
import { createPinia, setActivePinia } from 'pinia';
import { notificationsStore } from './state';

describe( 'notificationsStore notifyJobFinished', () => {
	beforeEach( () => {
		localStorage.clear();
		setActivePinia( createPinia() );
		const notificationMock = vi.fn();
		notificationMock.permission = 'granted';
		notificationMock.requestPermission = vi.fn();
		global.Notification = notificationMock;
	} );

	it( 'notifies once when tracked job finishes successfully', () => {
		const store = notificationsStore();
		store.jobIds = JSON.stringify( [ 22 ] );

		store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished successfully'
		} );
		expect( store.jobIds ).toBe( '[]' );
	} );

	it( 'does not notify when finished job is not tracked by this user', () => {
		const store = notificationsStore();
		store.jobIds = JSON.stringify( [ 22 ] );

		store.notifyJobFinished( {
			id: 23,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( Notification ).not.toHaveBeenCalled();
		expect( store.jobIds ).toBe( JSON.stringify( [ 22 ] ) );
	} );

	it( 'uses error text when tracked job finishes with a non-zero exit status', () => {
		const store = notificationsStore();
		store.jobIds = JSON.stringify( [ 22 ] );

		store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 1
		} );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig job finished', {
			body: 'Job 22 finished with errors'
		} );
		expect( store.jobIds ).toBe( '[]' );
	} );
} );

describe( 'notificationsStore with several jobs', () => {
	beforeEach( () => {
		localStorage.clear();
		setActivePinia( createPinia() );
		// Each notification carries its own close(), so a test can tell which
		// one was closed.
		const notificationMock = vi.fn( function () {
			this.close = vi.fn();
		} );
		notificationMock.permission = 'granted';
		notificationMock.requestPermission = vi.fn();
		global.Notification = notificationMock;
	} );

	it( 'keeps watching a job when another one starts', async () => {
		const store = notificationsStore();

		await store.setupUserNotificationsForJob( 22 );
		await store.setupUserNotificationsForJob( 23 );

		expect( store.jobIds ).toBe( JSON.stringify( [ 22, 23 ] ) );
	} );

	it( 'keeps watching the other jobs when one finishes', async () => {
		const store = notificationsStore();
		await store.setupUserNotificationsForJob( 22 );
		await store.setupUserNotificationsForJob( 23 );

		store.notifyJobFinished( {
			id: 22,
			finished_at: 1700000400,
			exit_status: 0
		} );

		expect( store.jobIds ).toBe( JSON.stringify( [ 23 ] ) );

		store.notifyJobFinished( {
			id: 23,
			finished_at: 1700000500,
			exit_status: 0
		} );

		expect( Notification ).toHaveBeenCalledTimes( 2 );
		expect( store.jobIds ).toBe( '[]' );
	} );

	it( 'notifies for a question of any watched job', async () => {
		const store = notificationsStore();
		await store.setupUserNotificationsForJob( 22 );
		await store.setupUserNotificationsForJob( 23 );

		store.notifyUser( { id: 1, job_id: 23, prompt: 'Continue?' } );

		expect( Notification ).toHaveBeenCalledWith( 'SpiderPig needs you!', {
			body: 'Job 23 requires user input',
			requireInteraction: true
		} );
	} );

	it( 'closes the notification of the job that was answered', async () => {
		const store = notificationsStore();
		await store.setupUserNotificationsForJob( 22 );
		await store.setupUserNotificationsForJob( 23 );

		store.notifyUser( { id: 1, job_id: 22, prompt: 'Continue?' } );
		store.notifyUser( { id: 2, job_id: 23, prompt: 'Continue?' } );
		const answered = store.userNotifications[ 22 ];
		const other = store.userNotifications[ 23 ];

		store.closeNotification( 22 );

		expect( answered.close ).toHaveBeenCalled();
		expect( other.close ).not.toHaveBeenCalled();
		expect( store.userNotifications[ 23 ] ).toBe( other );
	} );
} );
