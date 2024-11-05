import { defineStore } from 'pinia'
import { useLocalStorage } from "@vueuse/core"
import fakeJobrunner from './fakeJobrunner'

function makeApiUrl(url) {
    return new URL(url, window.location.origin)
}

const useAuthStore = defineStore('spiderpig-auth',
    {
        state() {
            return {
                token: useLocalStorage('spiderpig-auth-token', null),
                authFailing: false,
                testmodeData: useLocalStorage('spiderpig-testmode-data',
                    {
                        currentJob: null,
                        jobs: [],
                        lastJobId: 0,
                    })
            }
        },
        getters: {
            isAuthenticated(state) {
                return state.token !== null && !state.authFailing;
            },

            isTestMode() {
                return import.meta.env.DEV && import.meta.env.VITE_USE_TEST_MODE === "true";
            }
        },
        actions: {
            async call(url, options, decodeJson = true, signal = null) {
                if ( this.isTestMode ) {
                    throw new Error("Don't use call() in test mode")
                }

                let localOptions = {
                    ...options,
                    signal
                }

                if (this.token !== null) {
                    localOptions.headers = {
                        "Authorization": `Bearer ${this.token}`
                    }
                }

                const response = await fetch(makeApiUrl(url), localOptions)
                this.authFailing = (response.status == 401)

                if (!response.ok) {
                    throw new Error(`Response status: ${response.status}`);
                }

                if (decodeJson) {
                    return await response.json();
                } else {
                    return response;
                }
            },
            async login(username, password) {
                // Test-mode specific login behavior
                if ( this.isTestMode ) {
                    if (
                        username === import.meta.env.VITE_TEST_MODE_USERNAME &&
                        password === import.meta.env.VITE_TEST_MODE_PASSWORD
                    ) {
                        this.token = import.meta.env.VITE_TEST_MODE_TOKEN;
                        localStorage.setItem('spiderpig-auth-token', this.token);
                        return true;
                    } else {
                        this.authFailed = true
                        return { statusText: `Login failed: Unauthorized` }
                    }
                }

                const formData = new FormData()
                formData.append("username", username)
                formData.append("password", password)

                var response;

                try {
                    response = await fetch(makeApiUrl("/api/login"),
                        {
                            method: "POST",
                            body: formData,
                        }
                    )
                } catch (err) {
                    this.authFailing = true
                    return { statusText: `Login failed: ${err.message}` }
                }

                this.authFailing = (response.status === 401)

                if (!response.ok) {
                    if (response.status === 401) {
                        return response
                    }
                    throw new Error(`Response status: ${response.status}`);
                }

                const res = await response.json();

                this.token = res.token
                localStorage.setItem('spiderpig-auth-token', res.token)

                return true
            },
            logout() {
                this.token = null
                localStorage.removeItem('spiderpig-auth-token')
            },
            async getJobrunnerStatus() {
                if ( this.isTestMode ) {
                    return fakeJobrunner.getJobrunnerStatus();
                }

                return await this.call("/api/jobrunner/status");
            },
            async getLastNJobs(last = null) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.getLastNJobs(last)
                }

                const url = new URL("/api/jobs", window.location.origin);
                if (last !== null) {
                    url.searchParams.append("last", last)
                }
                return await this.call(url)
            },
            async getJobInfo(job_id) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.getJobInfo(job_id)
                }

                return await this.call(`/api/jobs/${job_id}`);
            },
            async respondInteraction(job_id, interaction_id, response) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.respondInteraction(job_id, interaction_id, response);
                }

                await this.call(
                    `/api/jobs/${job_id}/interact/${interaction_id}`,
                    {
                        method: "POST",
                        body: response,
                    }
                );
            },
            async signalJob(job_id, type) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.signalJob(job_id, type);
                }

                await this.call(`/api/jobs/${job_id}/signal/${type}`,
                    {
                        method: "POST",
                    }
                );
            },
            async getJobLog(job_id, abortsignal) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.getJobLog(job_id, abortsignal);
                }

                return await this.call(`/api/jobs/${job_id}/log`, {}, false, abortsignal);
            },
            async startBackport(change_urls) {
                if ( this.isTestMode ) {
                    return fakeJobrunner.createJob(["scap", "backport"].concat(change_urls));
                }

                const url = makeApiUrl("/api/jobs/backport")

                for (const change_url of change_urls) {
                    url.searchParams.append("change_url", change_url)
                }

                await this.call(url,
                    {
                        method: "POST",
                    }
                );
            },
            async startTrain() {
                await this.call("/api/jobs/train", {
                    method: "POST",
                })
            },
            async searchPatch( q, n ) {
                // @TODO: Move the testmode API request into a dedicated "fake" module
                if ( this.isTestMode ) {
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
                } else {
                    const url = makeApiUrl( "/api/searchPatch" );
                    const params = new URLSearchParams( { q, n } );
                    return await this.call( `${url}?${ params.toString() }` );
                }
            },
        },
    }
)

export default useAuthStore
