import fakeScapBackport from './fakeBackport.mjs';

async function sleep( ms ) {
	return new Promise( ( resolve ) => setTimeout( resolve, ms ) );
}

function timestamp() {
	return new Date().toISOString();
}

let nextJobId = 1;

class Job {
	id = null;

	queued_at = null;

	started_at = null;

	finished_at = null;

	user = null;

	command = null;

	exit_status = null;

	status = null;

	interaction = null;

	log = [];

	logIndex = 0;

	logWatcher = null;

	constructor( command ) {
		this.id = nextJobId++;
		this.queued_at = timestamp();
		this.command = JSON.stringify( command );
		this.user = 'testuser';
	}

	get commandDecoded() {
		return JSON.parse( this.command );
	}

	get commandPretty() {
		return JSON.parse( this.command ).join( ' ' );
	}

	watchLog( callback ) {
		this.logWatcher = callback;
	}

	unwatchLog() {
		this.logWatcher = null;
	}

	recordLog( line ) {
		this.log.push( line );
		this.logIndex++;

		const callback = this.logWatcher;
		if ( callback ) {
			callback( this.logIndex );
		}
	}

	setStatus( status ) {
		this.status = status;
	}
}

let nextInteractionId = 1;

class Interaction {
	id = null;

	job_id = null;

	constructor( job_id, payload ) {
		this.id = nextInteractionId++;
		this.job_id = job_id;

		for ( const key in payload ) {
			this[ key ] = payload[ key ];
		}
	}
}

class fakeApiserver {
	status = 'idle';

	currentJob = null;

	history = [];

	getJobrunnerStatus() {
		if ( this.currentJob ) {
			return {
				status: `Running job ${ this.currentJob.id }`,
				job_id: this.currentJob.id,
				pending_interaction: this.currentJob.interaction
			};
		} else {
			return {
				status: 'idle',
				job_id: null,
				pending_interaction: null
			};
		}
	}

	getJobs( limit, skip ) {
		// Returns jobs from newest to oldest
		let jobs = this.history;

		jobs = jobs.slice( -( limit + skip ), skip ? -skip : undefined );
		return { jobs: jobs.toReversed() };
	}

	#getJobById( job_id ) {
		// Returns a job object, or undefined if not found
		return this.history.find( ( job ) => job.id == job_id );
	}

	getJobInfo( job_id ) {
		const job = this.#getJobById( job_id );

		if ( !job ) {
			throw new Error( `Job with id ${ job_id } does not exist` );
		}

		return {
			job: job,
			pending_interaction: job.interaction
		};
	}

	respondInteraction( job_id, interaction_id, response ) {
		const job = this.#getJobById( job_id );

		if ( !job ) {
			throw new Error( `Job with id ${ job_id } does not exist` );
		}

		const interaction = job.interaction;

		if ( !interaction ) {
			throw new Error( `Job ${ job_id } does not have a pending interaction` );
		}

		interaction.resolve( response );
	}

	signalJob( job_id, type ) {
		const job = this.#getJobById( job_id );

		if ( !job ) {
			throw new Error( `Job with id ${ job_id } does not exist` );
		}

		job.signal( type );
	}

	getJobLog( job_id, abortsignal ) {
		// This function is expected to return an object that has a
		// "body" slot which conforms to the async iterable protocol
		const job = this.#getJobById( job_id );

		if ( !job ) {
			throw new Error( `Job with id ${ job_id } does not exist` );
		}

		async function waitForLogToChange() {
			try {
				await new Promise( ( resolve ) => job.watchLog( ( logIndex ) => resolve( logIndex ) ) );
			} finally {
				job.unwatchLog();
			}
		}

		return {
			body: {
				[ Symbol.asyncIterator ]() {
					const joblog = job.log;
					let index = 0;

					return {
						next() {
							return new Promise( async ( resolve ) => {
								while ( true ) {
									if ( index < joblog.length ) {
										return resolve( { value: joblog[ index++ ], done: false } );
									}

									// No more log lines available at this time.

									if ( job.finished_at ) {
										return resolve( { done: true } );
									} // The job is finished so there will be no more lines

									// The job is running so there could be more lines later.
									await waitForLogToChange();
									// Loop around and reprocess
								}
							} );
						}
					};
				}
			}
		};

	}

	async mainloop() {
		while ( true ) {
			const job = this.getNextUnstartedJob();

			if ( !job ) {
				await sleep( 2000 );
				continue;
			}

			try {
				this.currentJob = job;
				await this.runJob( job );
			} finally {
				this.currentJob = null;
			}
		}
	}

	createJob( command ) {
		const job = new Job( command );
		this.history.push( job );
		return job;
	}

	getNextUnstartedJob() {
		return this.history.find( ( job ) => !job.started_at );
	}

	async runJob( job ) {
		job.started_at = timestamp();
		console.log( `Running job ${ job.id }` );

		const bp = new fakeScapBackport();
		bp.on( 'status', ( status ) => job.setStatus( status ) );
		bp.on( 'log', ( line ) => job.recordLog( line ) );
		bp.on( 'interaction', async ( interaction ) => {
			interaction = new Interaction( job.id, interaction );
			job.interaction = interaction;
			// FIXME: Render the interaction (and the response) to the log
			const response = await new Promise( ( resolve ) => interaction.resolve = resolve );
			job.interaction = null;
			return response;
		} );
		await bp.run( job.commandDecoded.slice( 2 ) );

		console.log( `Job ${ job.id } finished` );
		job.finished_at = timestamp();
		job.exit_status = 0;
		job.status = `Job ${ job.id } terminated`;
	}

	async searchPatch( q, n ) {
		const url = 'https://gerrit.wikimedia.org/r/changes/';
		const params = new URLSearchParams( { q, n } );
		const response = await fetch( `${ url }?${ params.toString() }` );
		if ( response.ok ) {
			// Remove Gerrit's security prefix before transforming response to JSON
			const text = await response.text();
			const clean = text.replace( ")]}'", '' );
			const json = JSON.parse( clean );
			return json;
		} else {
			const text = await response.text();
			console.error( text );
			return [];
		}
	}
}

const apiserver = new fakeApiserver();
apiserver.mainloop();

export default apiserver;
