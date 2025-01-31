import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

const useAuthStore = defineStore( 'spiderpig-auth',
	{
		state() {
			return {
				token: useLocalStorage( 'spiderpig-auth-token', null ),
				authFailing: false
			};
		},
		getters: {
			isAuthenticated( state ) {
				return state.token !== null && !state.authFailing;
			},

			apiserverBaseURL() {
				return import.meta.env.VITE_APISERVER_URL ?
					import.meta.env.VITE_APISERVER_URL :
					window.location.origin;
			}
		},
		actions: {
			makeApiUrl( url ) {
				return new URL( url, this.apiserverBaseURL );
			},
			async call( url, options, decodeJson = true, signal = null ) {
				const localOptions = {
					...options,
					signal
				};

				if ( this.token !== null ) {
					localOptions.headers = {
						Authorization: `Bearer ${ this.token }`
					};
				}

				const response = await fetch( this.makeApiUrl( url ), localOptions );
				this.authFailing = ( response.status === 401 );

				if ( !response.ok ) {
					throw new Error( `Response status: ${ response.status }` );
				}

				if ( decodeJson ) {
					return await response.json();
				} else {
					return response;
				}
			},
			async login( username, password ) {
				const formData = new FormData();
				formData.append( 'username', username );
				formData.append( 'password', password );

				let response;

				try {
					response = await fetch( this.makeApiUrl( '/api/login' ),
						{
							method: 'POST',
							body: formData
						}
					);
				} catch ( err ) {
					this.authFailing = true;
					return { statusText: `Login failed: ${ err.message }` };
				}

				this.authFailing = ( response.status === 401 );

				if ( !response.ok ) {
					if ( response.status === 401 ) {
						return response;
					}
					throw new Error( `Response status: ${ response.status }` );
				}

				const res = await response.json();

				this.token = res.token;
				localStorage.setItem( 'spiderpig-auth-token', res.token );

				return true;
			},
			logout() {
				this.token = null;
				localStorage.removeItem( 'spiderpig-auth-token' );
			},
			async getJobrunnerStatus() {
				return await this.call( '/api/jobrunner/status' );
			},
			async getJobs( limit, skip ) {
				const url = new URL( '/api/jobs', this.apiserverBaseURL );
				url.searchParams.append( 'limit', limit );
				url.searchParams.append( 'skip', skip );
				return await this.call( url );
			},
			// eslint-disable-next-line camelcase
			async getJobInfo( job_id ) {
				// eslint-disable-next-line camelcase
				return await this.call( `/api/jobs/${ job_id }` );
			},
			// eslint-disable-next-line camelcase
			async respondInteraction( job_id, interaction_id, response ) {
				await this.call(
					// eslint-disable-next-line camelcase
					`/api/jobs/${ job_id }/interact/${ interaction_id }`,
					{
						method: 'POST',
						body: response
					}
				);
			},
			// eslint-disable-next-line camelcase
			async signalJob( job_id, type ) {
				// eslint-disable-next-line camelcase
				await this.call( `/api/jobs/${ job_id }/signal/${ type }`,
					{
						method: 'POST'
					}
				);
			},
			// eslint-disable-next-line camelcase
			async getJobLog( job_id, abortsignal, showSensitive = false ) {
				// eslint-disable-next-line camelcase
				const url = this.makeApiUrl( `/api/jobs/${ job_id }/log` );
				if ( showSensitive ) {
					url.searchParams.append( 'include_sensitive', true );
				}

				return await this.call( url, {}, false, abortsignal );
			},
			// eslint-disable-next-line camelcase
			async startBackport( change_urls ) {
				const url = this.makeApiUrl( '/api/jobs/backport' );

				// eslint-disable-next-line camelcase
				for ( const change_url of change_urls ) {
					url.searchParams.append( 'change_url', change_url );
				}

				await this.call( url,
					{
						method: 'POST'
					}
				);
			},
			async startTrain() {
				await this.call( '/api/jobs/train', {
					method: 'POST'
				} );
			},
			async searchPatch( q, n ) {
				const url = this.makeApiUrl( '/api/searchPatch' );
				const params = new URLSearchParams( { q, n } );
				return await this.call( `${ url }?${ params.toString() }` );
			}
		}
	}
);

export default useAuthStore;
