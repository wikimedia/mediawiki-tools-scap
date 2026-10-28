import { beforeEach, describe, expect, it, vi } from 'vitest';
import { mount } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import SpInteraction from './Interaction.vue';

const respondInteractionMock = vi.fn();
const notifyUserMock = vi.fn();
const closeNotificationMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		respondInteraction: respondInteractionMock
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

	it( 'notifies the user of each new interaction', async () => {
		const wrapper = await mountInteraction( '/jobs/42' );
		expect( notifyUserMock ).toHaveBeenCalledWith( revertInteraction );

		const nextInteraction = { ...revertInteraction, id: 6 };
		await wrapper.setProps( { interaction: nextInteraction } );

		expect( notifyUserMock ).toHaveBeenCalledTimes( 2 );
		expect( notifyUserMock ).toHaveBeenLastCalledWith( nextInteraction );
	} );
} );
