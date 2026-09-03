import { describe, it, expect, vi, beforeEach } from 'vitest';
import SpJobCard from './JobCard.vue';

const retryJobMock = vi.fn();
const setupUserNotificationsForJobMock = vi.fn();
const notifyJobFinishedMock = vi.fn();
const pushMock = vi.fn();
const queueIsBusyMock = vi.fn( () => false );

vi.mock( '../api', () => ( {
	default: () => ( {
		retryJob: retryJobMock
	} )
} ) );

// The named exports stay real, so the queue mapping under test is the true one.
vi.mock( '../jobrunner', async ( importOriginal ) => ( {
	...( await importOriginal() ),
	default: () => ( {
		idle: { value: true },
		busyQueues: { value: [] },
		runningJobs: { value: [] },
		queueIsBusy: queueIsBusyMock,
		mediawikiIsBusy: { value: false },
		runningJobOfType: () => null
	} )
} ) );

vi.mock( '../state', () => ( {
	notificationsStore: () => ( {
		setupUserNotificationsForJob: setupUserNotificationsForJobMock,
		notifyJobFinished: notifyJobFinishedMock
	} )
} ) );

vi.mock( 'vue-router', () => ( {
	RouterLink: {
		name: 'RouterLink',
		render: () => null
	},
	useRoute: () => ( {
		name: 'job'
	} ),
	useRouter: () => ( {
		push: pushMock
	} )
} ) );

vi.mock( 'vuetify/components/VIcon', () => ( {
	VIcon: {
		name: 'VIcon',
		render: () => null
	}
} ) );

const testJobProps = {
	id: 67,
	type: 'backport',
	queue_name: 'mediawiki',
	command_decoded: 'scap backport',
	user: 'tester',
	started_at: 1700000000,
	finished_at: 1700000060,
	exit_status: 1,
	status: {
		status: 'error',
		progress: null
	},
	interaction: null,
	data: {
		change_infos: []
	},
	duration: 60,
	running: false,
	orphaned: false
};

describe( 'SpJobCard confirmRetry', () => {
	beforeEach( () => {
		queueIsBusyMock.mockReset();
		queueIsBusyMock.mockReturnValue( false );
		retryJobMock.mockReset();
		setupUserNotificationsForJobMock.mockReset();
		notifyJobFinishedMock.mockReset();
		pushMock.mockReset();
	} );

	it( 'updates notification job tracking for retried jobs', async () => {
		retryJobMock.mockResolvedValue( { id: 91 } );
		setupUserNotificationsForJobMock.mockResolvedValue();

		const setupResult = SpJobCard.setup( testJobProps );

		await setupResult.confirmRetry();

		expect( retryJobMock ).toHaveBeenCalledWith( 67 );
		expect( setupUserNotificationsForJobMock ).toHaveBeenCalledWith( 91 );
		expect( pushMock ).toHaveBeenCalledWith( { name: 'job', params: { jobId: 91 } } );
	} );

	it( 'does not update notification job tracking if retry response has no id', async () => {
		retryJobMock.mockResolvedValue( {} );

		const setupResult = SpJobCard.setup( testJobProps );

		await setupResult.confirmRetry();

		expect( setupUserNotificationsForJobMock ).not.toHaveBeenCalled();
		expect( pushMock ).not.toHaveBeenCalled();
	} );
} );

describe( 'SpJobCard retry button', () => {
	beforeEach( () => {
		queueIsBusyMock.mockReset();
		queueIsBusyMock.mockReturnValue( false );
	} );

	it( 'offers retry for a finished backport job', () => {
		const setupResult = SpJobCard.setup( testJobProps );

		expect( setupResult.showRetryButton.value ).toBe( true );
		expect( setupResult.canRetry.value ).toBeTruthy();
		expect( setupResult.retryDisabledReason.value ).toBe( '' );
	} );

	it( 'offers retry for a finished deploy-service job', () => {
		const setupResult = SpJobCard.setup( {
			...testJobProps,
			type: 'deploy-service',
			queue_name: 'service:shellbox',
			data: { service: 'shellbox', message: 'bump image' }
		} );

		expect( setupResult.showRetryButton.value ).toBe( true );
		expect( setupResult.canRetry.value ).toBeTruthy();
		expect( setupResult.retryDisabledReason.value ).toBe( '' );
	} );

	// The apiserver answers 400 for this, so the button must not offer it.
	it( 'does not offer retry for a train job', () => {
		const setupResult = SpJobCard.setup( {
			...testJobProps,
			type: 'train',
			queue_name: 'mediawiki'
		} );

		expect( setupResult.showRetryButton.value ).toBe( false );
		expect( setupResult.canRetry.value ).toBeFalsy();
		expect( setupResult.retryDisabledReason.value ).toBe(
			'A train job cannot be retried'
		);
	} );
} );

describe( 'SpJobCard retry button while a queue is busy', () => {
	beforeEach( () => {
		queueIsBusyMock.mockReset();
		queueIsBusyMock.mockReturnValue( false );
	} );

	it( 'refuses a deploy-service retry while that service deploys', () => {
		queueIsBusyMock.mockImplementation( ( queue ) => queue === 'service:shellbox' );

		const setupResult = SpJobCard.setup( {
			...testJobProps,
			type: 'deploy-service',
			queue_name: 'service:shellbox',
			data: { service: 'shellbox' }
		} );

		expect( setupResult.canRetry.value ).toBeFalsy();
		expect( setupResult.retryDisabledReason.value ).toBe( 'shellbox is deploying' );
	} );

	it( 'allows a deploy-service retry while another service deploys', () => {
		queueIsBusyMock.mockImplementation( ( queue ) => queue === 'service:echostore' );

		const setupResult = SpJobCard.setup( {
			...testJobProps,
			type: 'deploy-service',
			queue_name: 'service:shellbox',
			data: { service: 'shellbox' }
		} );

		expect( setupResult.canRetry.value ).toBeTruthy();
		expect( setupResult.retryDisabledReason.value ).toBe( '' );
	} );

	it( 'refuses a backport retry while MediaWiki is deploying', () => {
		queueIsBusyMock.mockImplementation( ( queue ) => queue === 'mediawiki' );

		const setupResult = SpJobCard.setup( testJobProps );

		expect( setupResult.canRetry.value ).toBeFalsy();
		expect( setupResult.retryDisabledReason.value ).toBe( 'MediaWiki is deploying' );
	} );
} );
