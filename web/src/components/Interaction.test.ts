import { beforeEach, describe, expect, it, vi } from 'vitest';
import { flushPromises, mount } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import SpInteraction from './Interaction.vue';

const respondInteractionMock = vi.fn();
const getInteractionDetailsMock = vi.fn();
const notifyUserMock = vi.fn();
const closeNotificationMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		respondInteraction: respondInteractionMock,
		getInteractionDetails: getInteractionDetailsMock
	} )
} ) );

vi.mock( '@/state', () => ( {
	notificationsStore: () => ( {
		notifyUser: notifyUserMock,
		closeNotification: closeNotificationMock
	} )
} ) );

const stub = { template: '<div />' };

const router = createRouter( {
	history: createMemoryHistory(),
	routes: [
		{ path: '/', component: stub },
		{ name: 'job', path: '/jobs/:jobId', component: stub }
	]
} );

// Keep in sync with Backport._select_changes_to_revert in scap/backport.py
const REVERT_PROMPT =
	'The deployment was rolled back, but change 1359282 is still merged:\n' +
	'Prefetch the LAC revision_feature_counts artifact on group1 wikipedias ' +
	'(operations/mediawiki-config, master)\n\n' +
	'Revert change 1359282?';

const revertInteraction = {
	id: 5,
	job_id: 42,
	type: 'choices',
	prompt: REVERT_PROMPT,
	choices: { Yes: 'y', No: 'n', 'Exit without reverting': 'x' },
	default: 'y'
};

async function mountInteraction( path: string, interaction = revertInteraction ) {
	await router.replace( path );
	await router.isReady();

	return mount( SpInteraction, {
		props: { interaction },
		global: { plugins: [ router ] }
	} );
}

describe( 'SpInteraction', () => {
	beforeEach( () => {
		respondInteractionMock.mockReset();
		getInteractionDetailsMock.mockReset();
		notifyUserMock.mockReset();
		closeNotificationMock.mockReset();
	} );

	it( 'offers one button for each choice', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );

		expect( wrapper.findAll( 'button' ).map( ( button ) => button.text() ) ).toEqual(
			[ 'Yes', 'No', 'Exit without reverting' ]
		);
	} );

	it( 'sends the code of the choice that the user selects', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );

		const exitButton = wrapper.findAll( 'button' )
			.find( ( button ) => button.text() === 'Exit without reverting' );
		await exitButton.trigger( 'click' );

		expect( respondInteractionMock ).toHaveBeenCalledWith( 42, 5, 'x' );
		expect( closeNotificationMock ).toHaveBeenCalledWith( 42 );
	} );

	it( 'shows the prompt with no change on the page of its job', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );

		expect( wrapper.find( '.interaction__prompt' ).text() ).toBe( REVERT_PROMPT );
	} );

	it( 'adds a link to the job log on another page', async () => {
		const wrapper = await mountInteraction( '/' );
		const prompt = wrapper.find( '.interaction__prompt' );

		expect( prompt.text() ).toBe(
			`View the job log for details.\n\n${ REVERT_PROMPT }`
		);

		await prompt.find( 'a[data-job-log]' ).trigger( 'click' );
		await new Promise( ( resolve ) => setTimeout( resolve ) );

		expect( router.currentRoute.value.fullPath ).toBe( '/jobs/42#log' );
	} );

	it( 'puts the job log link after the report of a failure', async () => {
		const wrapper = await mountInteraction( '/', {
			...revertInteraction,
			prompt: 'The sync failed.\n\nRetry?'
		} );

		expect( wrapper.find( '.interaction__prompt' ).text() ).toBe(
			'The sync failed.\nView the job log for details.\n\nRetry?'
		);
	} );

	it( 'gets the details of a prompt only after the user asks for them', async () => {
		getInteractionDetailsMock.mockResolvedValue(
			{ details: '=== Diff for shellbox ===\n\x1b[32m+ replicas: 3\x1b[0m <b>' }
		);
		const wrapper = await mountInteraction( '/', {
			...revertInteraction,
			prompt: 'Continue with the deployment?',
			choices: { Yes: 'y', No: 'n' },
			details_title: 'Helmfile diffs',
			has_details: true
		} );

		// The details replace the job log link.
		expect( wrapper.find( '.interaction__prompt' ).text() ).toBe(
			'Continue with the deployment?'
		);
		const accordion = wrapper.find( 'details.interaction__details' );
		expect( accordion.find( 'summary' ).text() ).toBe( 'Helmfile diffs' );
		expect( wrapper.find( '.interaction__details__text' ).exists() ).toBe( false );
		expect( getInteractionDetailsMock ).not.toHaveBeenCalled();

		// The browser fires "toggle" when the user opens or closes the accordion.
		const toggle = async ( newState: string ) => {
			await accordion.trigger( 'toggle', { newState } );
			await flushPromises();
		};
		await toggle( 'open' );

		expect( getInteractionDetailsMock ).toHaveBeenCalledWith( 42, 5 );
		const details = wrapper.find( '.interaction__details__text' );
		expect( details.text() ).toBe( '=== Diff for shellbox ===\n+ replicas: 3 <b>' );
		expect( details.find( 'b' ).exists() ).toBe( false );

		// Close, then open again without a second request.
		await toggle( 'closed' );
		await toggle( 'open' );

		expect( getInteractionDetailsMock ).toHaveBeenCalledTimes( 1 );
	} );

	it( 'offers no details for a prompt without them', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );

		expect( wrapper.find( '.interaction__details' ).exists() ).toBe( false );
	} );

	it( 'notifies the user of each new interaction', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );
		expect( notifyUserMock ).toHaveBeenCalledWith( revertInteraction );

		const nextInteraction = { ...revertInteraction, id: 6 };
		await wrapper.setProps( { interaction: nextInteraction } );

		expect( notifyUserMock ).toHaveBeenCalledTimes( 2 );
		expect( notifyUserMock ).toHaveBeenLastCalledWith( nextInteraction );
	} );
} );
