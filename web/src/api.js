import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';

const useAuthStore = defineStore( 'spiderpig-auth',
	{
		state() {
			return {
				authUser: useLocalStorage( 'spiderpig-auth-user', null ),
				authFailing: false,
				authCode: null,
				authUrl: null
			};
		},
		getters: {
			isAuthenticated( state ) {
				return state.authUser && !state.authFailing;
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
					// console.log( `Redirecting to ${ newUrl }` );
					window.location.href = newUrl;
				}
				if ( authresp.code === 'needauth' ) {
					// console.log( `Setting window.location.href to ${ authresp.url }` );
					window.location.href = authresp.url;
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

				this.authFailing = false;

				if ( callOptions.checkOk !== false && !response.ok ) {
					throw new Error( `Response status: ${ response.status }` );
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
				const resp = await this.call( '/api/logout',
					{
						method: 'POST'
					}
				);
				this.authUser = null;
				localStorage.removeItem( 'spiderpig-auth-user' );
				return resp.SSOLogoutUrl;
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
