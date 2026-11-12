import { describe, expect, it } from 'vitest';
import { mount } from '@vue/test-utils';
import { CdxInfoChip } from '@wikimedia/codex';
import SpJobStatus from './JobStatus.vue';

function mountStopped( props: { startedAt?: number | null, exitStatus?: number | null, orphaned?: boolean } ) {
	return mount( SpJobStatus, {
		props: {
			status: { status: 'Job 1 finished', progress: null },
			running: false,
			startedAt: 1700000000,
			...props
		}
	} );
}

describe( 'SpJobStatus', () => {
	it( 'shows a successful job as a success', () => {
		const chip = mountStopped( { exitStatus: 0 } ).getComponent( CdxInfoChip );

		expect( chip.props( 'status' ) ).toBe( 'success' );
		expect( chip.text() ).toBe( 'Finished' );
	} );

	// The jobrunner records the exit status of the job's process.  A job
	// stopped with the STOP button exits with 130, and "Kill Job" gives -9.
	it.each( [ 1, 2, 130, -9 ] )( 'shows exit status %i as an error', ( exitStatus ) => {
		const chip = mountStopped( { exitStatus } ).getComponent( CdxInfoChip );

		expect( chip.props( 'status' ) ).toBe( 'error' );
		expect( chip.text() ).toBe( 'Error' );
	} );

	it( 'shows an orphaned job as a notice', () => {
		const chip = mountStopped( { exitStatus: null, orphaned: true } ).getComponent( CdxInfoChip );

		expect( chip.props( 'status' ) ).toBe( 'notice' );
		expect( chip.text() ).toBe( 'Orphaned' );
	} );

	it( 'shows a pending job as a notice', () => {
		const chip = mountStopped( { startedAt: null, exitStatus: null } ).getComponent( CdxInfoChip );

		expect( chip.props( 'status' ) ).toBe( 'notice' );
		expect( chip.text() ).toBe( 'Pending' );
	} );
} );
