import start_backport_script from './fakeBackportScripts/start_backport_script';
import build_and_sync_to_testservers_script from './fakeBackportScripts/build_and_sync_to_testservers_script';
import sync_remaining_script from './fakeBackportScripts/sync_remaining_script';

async function sleep( ms ) {
	return new Promise( ( resolve ) => setTimeout( resolve, ms ) );
}

function genLogMessageTimestamp() {
	return new Date().toLocaleTimeString( 'en-GB' );
}

class fakeScapBackport {
	eventHooks = {};

	status = null;

	// event must be 'status', 'log', or 'interaction'
	on( event, callback ) {
		this.eventHooks[ event ] = callback;
	}

	log( line, includeTimestamp = false ) {
		const callback = this.eventHooks.log;
		if ( callback ) {
			if ( includeTimestamp ) {
				line = genLogMessageTimestamp() + ' ' + line;
			}
		}
		callback( line );
	}

	async interact( interaction ) {
		const callback = this.eventHooks.interaction;
		if ( callback ) {
			const oldStatus = this.status;
			this.setStatus( 'Awaiting user interaction' );
			const resp = await callback( interaction );
			this.setStatus( oldStatus );
			return resp;
		}
	}

	setStatus( status ) {
		this.status = status;
		const callback = this.eventHooks.status;
		if ( callback ) {
			callback( status );
		}
	}

	async run( changeIds ) {
		if ( !changeIds ) {
			throw new Error( 'change_ids must be a list of strings' );
		}

		this.validateChangeIds( changeIds );
		if ( !await this.runBooleanScript( start_backport_script ) ) {
			return;
		}
		if ( !await this.runBooleanScript( build_and_sync_to_testservers_script ) ) {
			return;
		}
		await this.runScript( sync_remaining_script );
	}

	async runScript( script ) {
		for ( const entry of script ) {
			// FIXME: Need to be responsive to signals here
			await sleep( entry.gap * 1000 );
			if ( entry.type === 'line' ) {
				this.log( entry.line );
				continue;
			}
			if ( entry.type === 'status' ) {
				this.setStatus( entry.status );
				continue;
			}
			if ( entry.type === 'interaction' ) {
				return await this.interact( entry );
			}
		}
	}

	async runBooleanScript( script ) {
		const response = await this.runScript( script );

		if ( response === 'y' ) {
			return true;
		} else if ( response === 'n' ) {
			return false;
		} else {
			throw new Error( 'Invalid response' );
		}
	}

	validateChangeIds( changeIds ) {
		// FIXME: These messages should be green
		this.log( 'Checking whether requested changes are in a branch deployed to production and their dependencies valid...\n', true );
		for ( const changeId of changeIds ) {
			this.log( `Change '${ changeId }' validated for backport\n`, true );
		}
	}
}

export default fakeScapBackport;
