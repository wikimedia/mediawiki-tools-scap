import { beforeEach, describe, expect, it, vi } from 'vitest';
import { mount } from '@vue/test-utils';
import { createMemoryHistory, createRouter } from 'vue-router';
import { createVuetify } from 'vuetify';
import SpJobLog from './JobLog.vue';

const getJobLogMock = vi.fn();

vi.mock( '../api', () => ( {
	default: () => ( {
		getJobLog: getJobLogMock
	} )
} ) );

const stub = { template: '<div />' };

const router = createRouter( {
	history: createMemoryHistory(),
	routes: [
		{ name: 'job', path: '/jobs/:jobId', component: stub }
	]
} );

// Returns a response with a body that sends `chunks` one at a time.
function streamedResponse( chunks: string[] ) {
	const encoder = new TextEncoder();
	return {
		body: new ReadableStream( {
			start( controller ) {
				for ( const chunk of chunks ) {
					controller.enqueue( encoder.encode( chunk ) );
				}
				controller.close();
			}
		} )
	};
}

// The records of /api/jobs/{job_id}/log at the end of a backport that was
// rolled back.
const records = [
	{ type: 'connected', recno: 0 },
	{ type: 'line', line: '17:39:39 Failed scap sync-world: Backport for [[gerrit:1359282|Prefetch]]', recno: 1 },
	{ type: 'line', line: '\u001b[0m\u0007', recno: 2 },
	{ type: 'line', line: '<sensitive information hidden>', sensitive: true, recno: 3 },
	{
		type: 'line',
		line: 'The deployment was rolled back, but change 1359282 is still merged:\n\n' +
			'\u001b[1m[y] Yes\u001b[0m\n[n] No\n[x] Exit without reverting\n' +
			'Revert change 1359282? (default: [y]):',
		recno: 4
	},
	{ type: 'response', user: 'bruce', response: 'y', recno: 5 },
	{ type: 'EOF', recno: 5 }
];

async function mountJobLog() {
	await router.replace( '/jobs/42' );
	await router.isReady();

	return mount( SpJobLog, {
		props: { jobId: 42 },
		global: {
			plugins: [ router, createVuetify() ],
			stubs: { VTooltip: true }
		}
	} );
}

async function terminalText( wrapper ) {
	const terminal = wrapper.find( '.job-log__terminal' );
	await vi.waitFor( () => {
		expect( terminal.text() ).toContain( '[End of job log]' );
	} );
	return terminal.element.textContent;
}

describe( 'SpJobLog', () => {
	beforeEach( () => {
		getJobLogMock.mockReset();
	} );

	it( 'shows each line and response of the job', async () => {
		getJobLogMock.mockResolvedValue( streamedResponse(
			records.map( ( record ) => JSON.stringify( record ) + '\n' )
		) );

		const text = await terminalText( await mountJobLog() );

		expect( text.split( '\n' ) ).toEqual( [
			'17:39:39 Failed scap sync-world: Backport for [[gerrit:1359282|Prefetch]]',
			'\u0007',
			'<sensitive information hidden>',
			'The deployment was rolled back, but change 1359282 is still merged:',
			'',
			'[y] Yes',
			'[n] No',
			'[x] Exit without reverting',
			'Revert change 1359282? (default: [y]):',
			'[User \'bruce\' responded with \'y\']',
			'[End of job log]',
			''
		] );
		expect( getJobLogMock ).toHaveBeenCalledWith( 42, expect.anything(), false, 0 );
	} );

	it( 'joins a record that comes in more than one chunk', async () => {
		const stream = records.map( ( record ) => JSON.stringify( record ) + '\n' ).join( '' );
		const middleOfPrompt = stream.indexOf( 'Exit without' );
		getJobLogMock.mockResolvedValue( streamedResponse( [
			stream.slice( 0, middleOfPrompt ),
			stream.slice( middleOfPrompt )
		] ) );

		const text = await terminalText( await mountJobLog() );

		expect( text ).toContain( '[x] Exit without reverting\n' );
	} );

	it( 'resumes after the last record when the stream ends before EOF', async () => {
		const [ connected, firstLine, ...rest ] = records.map(
			( record ) => JSON.stringify( record ) + '\n'
		);
		getJobLogMock
			.mockResolvedValueOnce( streamedResponse( [ connected, firstLine ] ) )
			.mockResolvedValueOnce( streamedResponse( rest ) );

		const text = await terminalText( await mountJobLog() );

		expect( getJobLogMock ).toHaveBeenLastCalledWith( 42, expect.anything(), false, 1 );
		expect( text.match( /Failed scap sync-world/g ) ).toHaveLength( 1 );
	} );
} );
