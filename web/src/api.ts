import { defineStore } from 'pinia';
import { useLocalStorage } from '@vueuse/core';
import Job from './types/Job';
import JobrunnerStatus from './types/JobrunnerStatus';
import LinkSegment from './types/LinkSegment';
import TrainStatus from './types/TrainStatus';

// The body of a 401 response.  Keep in sync with authn_exception_handler() and
// login2() in scap/spiderpig/api.py.
interface AuthResponse {
	code: string
	message: string
	url?: string
	user?: string
}

// Keep in sync with whoami() in scap/spiderpig/api.py.
export interface WhoAmI {
	user: string | null
	fully_authenticated?: boolean
	groups?: string[]
	isAuthorized?: boolean
	isAdmin?: boolean
	otpHost?: string
	loginUrl?: string
}

// Keep in sync with get_jobs() in scap/spiderpig/api.py.
export interface JobList {
	jobs: Job[]
}

// Keep in sync with get_job() in scap/spiderpig/api.py.
export interface JobInfo {
	job: Job
}

// Keep in sync with get_errors() in scap/spiderpig/api.py.
export interface ErrorLogs {
	message: string
	log: Record<string, {
		count: number
		versions: string[]
		note?: {
			text: string
			linkified: Array<string | LinkSegment>
			user: string
			updatedAt: number
		}
	}>
}

// Keep in sync with get_error_total() in scap/spiderpig/api.py.
export interface ErrorTotal {
	message: string
	total: number
}

// The response of each route that creates a job.  Keep in sync with start_train(),
// start_deploy_service(), _create_backport_job() and retry_job() in
// scap/spiderpig/api.py.
export interface CreatedJob {
	message: string
	id: number
}

// Keep in sync with get_services() in scap/spiderpig/api.py.
export interface Services {
	services: string[]
	// The error of each service whose catalog entry has one, by service name.
	errors: Record<string, string>
}

// One Gerrit ChangeInfo record.  Keep in sync with searchPatch() in
// scap/spiderpig/api.py.  It has more fields than these.
export interface GerritChange {
	_number: number
	subject: string
}

// The options of call().
interface CallOptions {
	// Redirect to the login page for a 401.  The default is true.
	redirectOn401?: boolean
	// Throw an ApiError for a response that is not OK.  The default is true.
	checkOk?: boolean
	// Return the decoded JSON body instead of the Response.  The default is true.
	decodeJson?: boolean
}

export class ApiError extends Error {
	response: Response;
	// The JSON body of the response, or null when the body is not JSON.  An
	// HTTPException of scap/spiderpig/api.py puts its detail here.
	respJson: { detail?: { message?: string } } | null;

	constructor(
		message: string,
		properties: { response: Response, respJson: ApiError['respJson'] }
	) {
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
				authUser: useLocalStorage<string | null>( 'spiderpig-auth-user', null ),
				windowLocationSet: false
			};
		},
		getters: {
			apiserverBaseURL(): string {
				return import.meta.env.VITE_APISERVER_URL ?
					import.meta.env.VITE_APISERVER_URL :
					window.location.origin;
			}
		},
		actions: {
			makeApiUrl( url: string | URL ): URL {
				return new URL( url, this.apiserverBaseURL );
			},

			setWindowLocation( url: string ): void {
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
				} catch {
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

			async handle401(
				url: string | URL,
				response: Response,
				doSetWindowLocation = true
			): Promise<never> {
				const authresp: AuthResponse = await response.json();

				this.authUser = authresp.user ?? null;
				localStorage.setItem( 'spiderpig-auth-user', String( this.authUser ) );

				console.log( `handle401: Auth failed for call to ${ url }: code: ${ authresp.code }, message: ${ authresp.message }` );

				if ( authresp.code === 'need2fa' ) {
					const params = new URLSearchParams( { redirect: window.location.href } );
					const newUrl = `/login?${ params.toString() }`;
					if ( doSetWindowLocation ) {
						this.setWindowLocation( newUrl );
					}
				}
				if ( authresp.code === 'needauth' && authresp.url ) {
					if ( doSetWindowLocation ) {
						this.setWindowLocation( authresp.url );
					}
				}

				throw new Error( `Auth failed for call to ${ url }: ${ authresp.message }` );
			},

			// Returns the decoded JSON body, or the Response when callOptions.decodeJson is
			// false.  Each caller states the type that it expects.
			async call(
				url: string | URL,
				fetchOptions: RequestInit = {},
				callOptions: CallOptions = {}
			): Promise<any> {
				const augmentedFetchOptions: RequestInit = {
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
					} catch {
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
			async post(
				url: string,
				obj?: unknown,
				fetchOptions: RequestInit = {},
				callOptions: CallOptions = {}
			): Promise<any> {
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
			async login2( otp: string ): Promise<AuthResponse> {
				const response: Response = await this.call(
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

				let resp: AuthResponse;

				if ( response.ok || response.status === 400 || response.status === 429 ) {
					resp = await response.json();
					this.authUser = resp.user ?? null;
					localStorage.setItem( 'spiderpig-auth-user', String( this.authUser ) );
					return resp;
				}

				throw new Error( `/api/2fa response status: ${ response.status }` );

			},
			async logout(): Promise<string> {
				return await this.logoutCommon( '/api/logout' );
			},
			async logoutAll(): Promise<string> {
				return await this.logoutCommon( '/api/logoutAll' );
			},
			// Returns the SSOLogoutUrl of logout() or logoutAll() in scap/spiderpig/api.py.
			async logoutCommon( url: string ): Promise<string> {
				const resp = await this.call( url,
					{
						method: 'POST'
					}
				);
				this.authUser = null;
				localStorage.removeItem( 'spiderpig-auth-user' );
				return resp.SSOLogoutUrl;

			},
			async whoami(): Promise<WhoAmI> {
				return await this.call( '/api/whoami' );
			},
			async getEvents( abortsignal: AbortSignal ): Promise<Response> {
				return await this.call(
					'/api/events',
					{ signal: abortsignal },
					{ decodeJson: false }
				);
			},
			async getJobrunnerStatus(): Promise<JobrunnerStatus> {
				return await this.call( '/api/jobrunner/status' );
			},
			async getJobs( limit: number, skip: number, type: string | null = null ): Promise<JobList> {
				const url = new URL( '/api/jobs', this.apiserverBaseURL );
				url.searchParams.append( 'limit', String( limit ) );
				url.searchParams.append( 'skip', String( skip ) );
				if ( type ) {
					url.searchParams.append( 'type', type );
				}
				return await this.call( url );
			},
			async getJobInfo( job_id: number | string ): Promise<JobInfo> {
				return await this.call( `/api/jobs/${ job_id }` );
			},
			async getLogs(): Promise<ErrorLogs> {
				return await this.call( '/api/monitoring/logs/mediawiki' );
			},
			async setErrorNote( message: string, note: string ): Promise<void> {
				await this.post(
					'/api/monitoring/logs/mediawiki/note',
					{ message, note }
				);
			},
			async getLogsTotal(): Promise<ErrorTotal> {
				// Setting redirectOn401 here avoids the total widget causing
				// a redirect loop for users who haven't entered their OTP yet:
				return await this.call( '/api/monitoring/logs/mediawiki/total',
					{},
					{ redirectOn401: false }
				);
			},
			async respondInteraction(
				job_id: number,
				interaction_id: number,
				response: string
			): Promise<void> {
				await this.call(
					`/api/jobs/${ job_id }/interact/${ interaction_id }`,
					{
						method: 'POST',
						body: response
					}
				);
			},
			// Returns the response of get_interaction_details() in scap/spiderpig/api.py.
			async getInteractionDetails(
				job_id: number,
				interaction_id: number
			): Promise<{ details: string | null }> {
				return await this.call( `/api/jobs/${ job_id }/interact/${ interaction_id }/details` );
			},
			async signalJob( job_id: number | string, type: string ): Promise<void> {
				await this.call( `/api/jobs/${ job_id }/signal/${ type }`,
					{
						method: 'POST'
					}
				);
			},
			async getJobLog(
				job_id: number | string,
				abortsignal: AbortSignal,
				showSensitive = false,
				resumeRecord = 0
			): Promise<Response> {
				const url = this.makeApiUrl( `/api/jobs/${ job_id }/log` );
				if ( showSensitive ) {
					url.searchParams.append( 'include_sensitive', 'true' );
				}
				if ( resumeRecord > 0 ) {
					url.searchParams.append( 'resume_record', String( resumeRecord ) );
				}

				return await this.call(
					url,
					{ signal: abortsignal },
					{ decodeJson: false }
				);
			},
			async startBackport( change_urls: string[] ): Promise<CreatedJob> {
				const url = this.makeApiUrl( '/api/jobs/backport' );

				for ( const change_url of change_urls ) {
					url.searchParams.append( 'change_url', change_url );
				}

				return await this.call( url,
					{
						method: 'POST'
					}
				);
			},
			async startTrain( promotion: object ): Promise<CreatedJob> {
				return await this.post( '/api/jobs/train', promotion );
			},
			async getServices(): Promise<Services> {
				return await this.call( '/api/services' );
			},
			async startDeployService( deployment: object ): Promise<CreatedJob> {
				return await this.post( '/api/jobs/deploy-service', deployment );
			},
			async retryJob( job_id: number ): Promise<CreatedJob> {
				return await this.post( `/api/jobs/${ job_id }/retry` );
			},
			async trainStatus(): Promise<TrainStatus> {
				return await this.call( '/api/train/status' );
			},
			async searchPatch( changeNumOrUrl: string ): Promise<GerritChange[]> {
				const url = this.makeApiUrl( '/api/searchPatch' );
				const params = new URLSearchParams( { changeNumOrUrl } );
				return await this.call( `${ url }?${ params.toString() }` );
			}
		}
	}
);

export default useAuthStore;
