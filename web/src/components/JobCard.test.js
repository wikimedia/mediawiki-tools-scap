import { describe, it, expect, vi, beforeEach } from 'vitest';

const retryJobMock = vi.fn();
const setupUserNotificationsForJobMock = vi.fn();
const notifyJobFinishedMock = vi.fn();
const pushMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		retryJob: retryJobMock
	} )
} ) );

vi.mock( '../jobrunner', () => ( {
	default: () => ( {
		idle: { value: true }
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
		retryJobMock.mockReset();
		setupUserNotificationsForJobMock.mockReset();
		notifyJobFinishedMock.mockReset();
		pushMock.mockReset();
	} );

	it( 'updates notification job tracking for retried jobs', async () => {
		retryJobMock.mockResolvedValue( { id: 91 } );
		setupUserNotificationsForJobMock.mockResolvedValue();

		const module = await import( './JobCard.vue' );
		const setupResult = module.default.setup( testJobProps );

		await setupResult.confirmRetry();

		expect( retryJobMock ).toHaveBeenCalledWith( 67 );
		expect( setupUserNotificationsForJobMock ).toHaveBeenCalledWith( 91 );
		expect( pushMock ).toHaveBeenCalledWith( { name: 'job', params: { jobId: 91 } } );
	} );

	it( 'does not update notification job tracking if retry response has no id', async () => {
		retryJobMock.mockResolvedValue( {} );

		const module = await import( './JobCard.vue' );
		const setupResult = module.default.setup( testJobProps );

		await setupResult.confirmRetry();

		expect( setupUserNotificationsForJobMock ).not.toHaveBeenCalled();
		expect( pushMock ).not.toHaveBeenCalled();
	} );
} );
