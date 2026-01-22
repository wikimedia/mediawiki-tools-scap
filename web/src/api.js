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
				// Avoid redirecting if we've already redirected once this session
				// or we're already on the exact target URL. We build a canonical
				// absolute URL for comparison using the current origin. This handles
				// relative paths (e.g. '/notauthorized') and absolute inputs.
				let targetHref;
				try {
					// new URL(relative, base) will resolve relative paths correctly.
					// If url is already absolute, the base is ignored.
					const targetUrl = new URL( url, window.location.origin );
					targetHref = targetUrl.href;
				} catch ( e ) {
					// If URL construction fails, fall back to original string
					targetHref = url;
				}
				if ( this.windowLocationSet || window.location.href === targetHref ) {
					// console.log(
					//   `setWindowLocation: Not setting window.location.href to ${ url } ` +
					//   'because it was already set or we are already on that page'
					// );
					return;
				}
				// console.log( `Setting window.location.href to ${ url }` );
				this.windowLocationSet = true;
				window.location.href = url;
			},

			async handle401( url, response, doSetWindowLocation = true ) {
				const authresp = await response.json();

				this.authUser = authresp.user;
				localStorage.setItem( 'spiderpig-auth-user', this.authUser );

				console.log( `handle401: Auth failed for call to ${ url }: code: ${ authresp.code }, message: ${ authresp.message }` );

				if ( authresp.code === 'need2fa' ) {
					const params = new URLSearchParams( { redirect: window.location.href } );
					const newUrl = `/login?${ params.toString() }`;
					if ( doSetWindowLocation ) {
						this.setWindowLocation( newUrl );
					}
				}
				if ( authresp.code === 'needauth' ) {
					if ( doSetWindowLocation ) {
						this.setWindowLocation( authresp.url );
					}
				}

				throw new Error( `Auth failed for call to ${ url }: ${ authresp.message }` );
			},

			async call( url, fetchOptions = {}, callOptions = {} ) {
				const augmentedFetchOptions = {
					...fetchOptions,
					credentials: import.meta.env.VITE_APISERVER_URL ? 'include' : 'same-origin'
				};

				callOptions = {
					redirectOn401: true,
					...callOptions
				};

				const response = await fetch( this.makeApiUrl( url ), augmentedFetchOptions );

				if ( response.status === 401 ) {
					// This will throw an error
					await this.handle401( url, response, callOptions.redirectOn401 );
				}

				if ( response.status === 403 ) {
					this.setWindowLocation( '/notauthorized' );
				}

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
			async post( url, obj, fetchOptions = {}, callOptions = {} ) {
				return await this.call(
					url,
					{
						method: 'POST',
						body: JSON.stringify( obj ),
						headers: {
							'Content-Type': 'application/json'
						},
						...fetchOptions
					},
					callOptions
				);
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
			async getLogsTotal() {
				// Setting redirectOn401 here avoids the total widget causing
				// a redirect loop for users who haven't entered their OTP yet:
				return await this.call( '/api/monitoring/logs/mediawiki/total',
					{},
					{ redirectOn401: false }
				);
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
			async startTrain( promotion ) {
				return await this.post( '/api/jobs/train', promotion );
			},
			// eslint-disable-next-line camelcase
			async retryJob( job_id ) {
				// eslint-disable-next-line camelcase
				return await this.post( `/api/jobs/${ job_id }/retry` );
			},
			async trainStatus() {
				return await this.call( '/api/train/status' );
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
