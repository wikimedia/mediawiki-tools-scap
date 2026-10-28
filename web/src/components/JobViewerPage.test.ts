import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { flushPromises, mount } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import { createVuetify } from 'vuetify';
import JobViewerPage from './JobViewerPage.vue';

const getJobInfoMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		getJobInfo: getJobInfoMock
	} )
} ) );

const stub = { template: '<div />' };

const router = createRouter( {
	history: createMemoryHistory(),
	routes: [
		{ name: 'job', path: '/jobs/:jobId', component: stub }
	]
} );

const runningJob = {
	id: 42,
	type: 'backport',
	command: '["scap", "backport", "1359282"]',
	finished_at: null,
	running: true
};

const revertInteraction = {
	id: 5,
	job_id: 42,
	type: 'choices',
	prompt: 'Revert change 1359282?',
	choices: { Yes: 'y', No: 'n', 'Exit without reverting': 'x' },
	default: 'y'
};

async function mountJobViewerPage() {
	await router.replace( '/jobs/42' );
	await router.isReady();

	const wrapper = mount( JobViewerPage, {
		props: { jobId: '42' },
		global: {
			plugins: [ router, createVuetify() ],
			stubs: { SpInteraction: true, SpJobCard: true }
		}
	} );
	await flushPromises();
	return wrapper;
}

function shownInteractions( wrapper ) {
	return wrapper.findAllComponents( { name: 'SpInteraction' } )
		.map( ( interaction ) => interaction.props( 'interaction' ) );
}

describe( 'JobViewerPage', () => {
	beforeEach( () => {
		vi.useFakeTimers();
		getJobInfoMock.mockReset();
	} );

	afterEach( () => {
		vi.useRealTimers();
	} );

	it( 'shows the pending interaction above and below the job', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: runningJob,
			pending_interaction: revertInteraction
		} );

		const wrapper = await mountJobViewerPage();

		expect( getJobInfoMock ).toHaveBeenCalledWith( '42' );
		expect( shownInteractions( wrapper ) ).toEqual(
			[ revertInteraction, revertInteraction ]
		);
		expect( wrapper.find( '#bottom-interaction' ).exists() ).toBe( true );
	} );

	it( 'shows an interaction that arrives while the page is open', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: runningJob,
			pending_interaction: null
		} );
		const wrapper = await mountJobViewerPage();
		expect( shownInteractions( wrapper ) ).toEqual( [] );

		getJobInfoMock.mockResolvedValue( {
			job: runningJob,
			pending_interaction: revertInteraction
		} );
		await vi.advanceTimersByTimeAsync( 1000 );

		expect( shownInteractions( wrapper ) ).toEqual(
			[ revertInteraction, revertInteraction ]
		);
	} );

	it( 'removes the interaction after the response', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: runningJob,
			pending_interaction: revertInteraction
		} );
		const wrapper = await mountJobViewerPage();

		getJobInfoMock.mockResolvedValue( {
			job: runningJob,
			pending_interaction: null
		} );
		await vi.advanceTimersByTimeAsync( 1000 );

		expect( shownInteractions( wrapper ) ).toEqual( [] );
	} );

	it( 'stops asking for the job after the job finishes', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, finished_at: 1793125946, running: false },
			pending_interaction: null
		} );
		await mountJobViewerPage();

		await vi.advanceTimersByTimeAsync( 5000 );

		expect( getJobInfoMock ).toHaveBeenCalledTimes( 1 );
	} );
} );
