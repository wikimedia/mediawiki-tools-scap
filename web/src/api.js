import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

class ApiError extends Error {
	constructor( message, properties ) {
		super( message );
		this.name = 'ApiError';
		this.response = properties.response;
		this.respJson = properties.respJson;
	}
}

const useAuthStore = defineStore( 'spiderpig-auth',
	{
		state() {
			return {
				authUser: useLocalStorage( 'spiderpig-auth-user', null ),
				authFailing: false, // FIXME: This is set in several places but never read.
				authCode: null,
				authUrl: null,
				windowLocationSet: false
			};
		},
		getters: {
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

			setWindowLocation( url ) {
				if ( this.windowLocationSet ) {
					// eslint-disable-next-line max-len
					// console.log( `setWindowLocation: Not setting window.location.href to ${ url } because it was already set` );
					return;
				}
				// console.log( `Setting window.location.href to ${ url }` );
				this.windowLocationSet = true;
				window.location.href = url;
			},

			async handle401( url, response ) {
				const authresp = await response.json();

				this.authFailing = true;
				this.authCode = authresp.code;
				this.authUrl = authresp.url;
				this.authUser = authresp.user;
				localStorage.setItem( 'spiderpig-auth-user', this.authUser );

				console.log( `handle401: Auth failed for call to ${ url }: code: ${ authresp.code }, message: ${ authresp.message }` );

				if ( authresp.code === 'need2fa' ) {
					const params = new URLSearchParams( { redirect: window.location.href } );
					const newUrl = `/login?${ params.toString() }`;
					this.setWindowLocation( newUrl );
				}
				if ( authresp.code === 'needauth' ) {
					this.setWindowLocation( authresp.url );
				}

				throw new Error( `Auth failed for call to ${ url }: ${ authresp.message }` );
			},

			async call( url, fetchOptions = {}, callOptions = {} ) {
				const augmentedFetchOptions = {
					...fetchOptions,
					credentials: import.meta.env.VITE_APISERVER_URL ? 'include' : 'same-origin'
				};

				const response = await fetch( this.makeApiUrl( url ), augmentedFetchOptions );

				if ( response.status === 401 ) {
					// This will throw an error
					await this.handle401( url, response );
				}

				if ( response.status === 403 ) {
					this.setWindowLocation( '/notauthorized' );
				}

				this.authFailing = false;

				if ( callOptions.checkOk !== false && !response.ok ) {
					let respJson = null;
					try {
						respJson = await response.json();
					} catch ( e ) {
						// Ignore JSON parsing error
					}

					throw new ApiError( `HTTP request failed with status: ${ response.status }`,
						{ response, respJson }
					);
				}

				if ( callOptions.decodeJson !== false ) {
					return await response.json();
				} else {
					return response;
				}
			},
			async login2( otp ) {
				const response = await this.call(
					'/api/2fa',
					{
						method: 'POST',
						body: otp
					},
					{
						checkOk: false,
						decodeJson: false
					}
				);

				let resp;

				if ( response.ok || response.status === 400 ) {
					resp = await response.json();
					this.authUser = resp.user;
					localStorage.setItem( 'spiderpig-auth-user', this.authUser );
					this.authFailing = ( resp.code !== 'ok' );
					return resp;
				}

				throw new Error( `/api/2fa response status: ${ response.status }` );

			},
			async logout() {
				return await this.logoutCommon( '/api/logout' );
			},
			async logoutAll() {
				return await this.logoutCommon( '/api/logoutAll' );
			},
			async logoutCommon( url ) {
				const resp = await this.call( url,
					{
						method: 'POST'
					}
				);
				this.authUser = null;
				localStorage.removeItem( 'spiderpig-auth-user' );
				return resp.SSOLogoutUrl;

			},
			async whoami() {
				return await this.call( '/api/whoami' );
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
			async getLogs() {
				return await this.call( '/api/monitoring/logs/mediawiki' );
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

				return await this.call(
					url,
					{ signal: abortsignal },
					{ decodeJson: false }
				);
			},
			// eslint-disable-next-line camelcase
			async startBackport( change_urls ) {
				const url = this.makeApiUrl( '/api/jobs/backport' );

				// eslint-disable-next-line camelcase
				for ( const change_url of change_urls ) {
					url.searchParams.append( 'change_url', change_url );
				}

				return await this.call( url,
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
			async searchPatch( changeNumOrUrl ) {
				const url = this.makeApiUrl( '/api/searchPatch' );
				const params = new URLSearchParams( { changeNumOrUrl } );
				return await this.call( `${ url }?${ params.toString() }` );
			}
		}
	}
);

export default useAuthStore;
