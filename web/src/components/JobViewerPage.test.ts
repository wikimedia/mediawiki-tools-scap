import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { enableAutoUnmount, flushPromises, mount, VueWrapper } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import { createVuetify } from 'vuetify';
import JobViewerPage from './JobViewerPage.vue';
import { unfinishedJobDigests } from '../events';

const getJobInfoMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		getJobInfo: getJobInfoMock,
		// The tests set unfinishedJobDigests instead of using the event stream.
		getEvents: () => new Promise( () => {} )
	} )
} ) );

// A page that stayed mounted after its test would refetch when a later test sets
// unfinishedJobDigests.
enableAutoUnmount( afterEach );

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
	command_decoded: 'scap backport 1359282',
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

function shownInteractions( wrapper: VueWrapper ) {
	return wrapper.findAllComponents( { name: 'SpInteraction' } )
		.map( ( interaction ) => interaction.props( 'interaction' ) );
}

describe( 'JobViewerPage', () => {
	beforeEach( () => {
		getJobInfoMock.mockReset();
		unfinishedJobDigests.value = { 42: 'queued' };
	} );

	it( 'shows the pending interaction above and below the job', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: revertInteraction }
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
			job: { ...runningJob, interaction: null }
		} );
		const wrapper = await mountJobViewerPage();
		expect( shownInteractions( wrapper ) ).toEqual( [] );

		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: revertInteraction }
		} );
		unfinishedJobDigests.value = { 42: 'prompted' };
		await flushPromises();

		expect( shownInteractions( wrapper ) ).toEqual(
			[ revertInteraction, revertInteraction ]
		);
	} );

	it( 'removes the interaction after the response', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: revertInteraction }
		} );
		const wrapper = await mountJobViewerPage();

		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: null }
		} );
		unfinishedJobDigests.value = { 42: 'responded' };
		await flushPromises();

		expect( shownInteractions( wrapper ) ).toEqual( [] );
	} );

	it( 'ignores changes to other jobs', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: null }
		} );
		await mountJobViewerPage();

		unfinishedJobDigests.value = { 42: 'queued', 43: 'queued' };
		await flushPromises();

		expect( getJobInfoMock ).toHaveBeenCalledTimes( 1 );
	} );

	it( 'shows the job after it finishes', async () => {
		getJobInfoMock.mockResolvedValue( {
			job: { ...runningJob, interaction: null }
		} );
		const wrapper = await mountJobViewerPage();

		const finishedJob = {
			...runningJob, finished_at: 1793125946, running: false, interaction: null
		};
		getJobInfoMock.mockResolvedValue( { job: finishedJob } );
		unfinishedJobDigests.value = {};
		await flushPromises();

		expect( wrapper.findComponent( { name: 'SpJobCard' } ).props() )
			.toMatchObject( { finished_at: 1793125946 } );
		expect( getJobInfoMock ).toHaveBeenCalledTimes( 2 );
	} );
} );
