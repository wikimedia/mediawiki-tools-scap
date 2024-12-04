import start_backport_script from './fakeBackportScripts/start_backport_script';
import build_and_sync_to_testservers_script from './fakeBackportScripts/build_and_sync_to_testservers_script';
import sync_remaining_script from './fakeBackportScripts/sync_remaining_script';

function genLogMessageTimestamp() {
	return new Date().toLocaleTimeString( 'en-GB' );
}

class fakeScapBackport {
	eventHooks = {};

	status = null;

	pendingSignal = null;

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

			if ( resp.signal ) {
				this.setStatus( `Job was ${ resp.signal }ed` );
				throw resp;
			}

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

	async sleep( ms ) {
		const p = new Promise( ( resolve, error ) => {
			this.sleepError = error;
			this.sleepTimeout = setTimeout( resolve, ms );
		} );

		await p;

		if ( this.sleepTimeout ) {
			clearTimeout( this.sleepTimeout );
			this.sleepTimeout = null;
			this.sleepError = null;
		}
	}

	signal( type ) {
		this.pendingSignal = type;

		if ( this.sleepTimeout ) {
			clearTimeout( this.sleepTimeout );
			this.sleepTimeout = null;

			this.sleepError( { signal: type } );
		}
	}

	// Returns the exit_status
	async run( changeIds ) {
		if ( !changeIds ) {
			throw new Error( 'change_ids must be a list of strings' );
		}

		this.validateChangeIds( changeIds );
		try {
			if ( !await this.runBooleanScript( start_backport_script ) ) {
				return 0;
			}
			if ( !await this.runBooleanScript( build_and_sync_to_testservers_script ) ) {
				return 0;
			}
			await this.runScript( sync_remaining_script );
			return 0;
		} catch ( e ) {
			if ( e.signal ) {
				this.log( `[${ e.signal }ed by user]` );

				if ( e.signal === 'interrupt' ) {
					return -2;
				} else if ( e.signal === 'kill' ) {
					return -9;
				} else {
					throw new Error( `Unexpected signal type: ${ e.signal }` );
				}
			} else {
				console.log( 'Caught unexpected exception:', e );
				return 1;
			}
		}
	}

	// This returns the value of the first interaction
	// seen in `script`.  If no interaction is seen, the
	// return value is undefined.
	async runScript( script ) {
		for ( const entry of script ) {
			await this.sleep( entry.gap * 1000 );

			if ( this.pendingSignal ) {
				throw { signal: this.pendingSignal };
			}

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
			throw new Error( `Invalid response: ${ response }` );
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
