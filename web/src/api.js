import { defineStore } from 'pinia'
import { useLocalStorage } from "@vueuse/core"
import fakeJobrunner from './fakeJobrunner'

const API_BASE_URL = "http://localhost:8001"

// Testmode problems:
// * each function needs to check isAuthenticated before returning
//   test data.... and must return an error if not...
const TESTMODE = false

function makeApiUrl(url) {
    return new URL(url, API_BASE_URL);
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
            isAuthenticated(state) { return state.token !== null && !state.authFailing }
        },
        actions: {
            async call(url, options, decodeJson = true, signal = null) {
                if (TESTMODE) {
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

                if (TESTMODE) {
                    if (username == "scappy" && password == "123") {
                        this.authFailed = false
                        this.token = "secret"
                        localStorage.setItem('spiderpig-auth-token', this.token)
                        return true
                    }
                    this.authFailed = true
                    return { statusText: `Login failed: Unauthorized` }
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
                this.authFailing = (response.status == 401)

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
                if (TESTMODE)
                    return fakeJobrunner.getJobrunnerStatus()

                return await this.call("/api/jobrunner/status");
            },
            async getLastNJobs(last = null) {
                if (TESTMODE)
                    return fakeJobrunner.getLastNJobs(last)

                const url = new URL("/api/jobs", API_BASE_URL)
                if (last !== null) {
                    url.searchParams.append("last", last)
                }
                return await this.call(url)
            },
            async getJobInfo(job_id) {
                if (TESTMODE)
                    return fakeJobrunner.getJobInfo(job_id)

                return await this.call(`/api/jobs/${job_id}`);
            },
            async getJobInteraction(job_id) {
                if (TESTMODE)
                    return fakeJobrunner.getJobInteraction(job_id)

                const job_info = await this.getJobInfo(job_id);
                return job_info.pending_interaction;
            },
            async respondInteraction(job_id, interaction_id, response) {
                if (TESTMODE) {
                    return fakeJobrunner.respondInteraction(job_id, interaction_id, response)
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
                if (TESTMODE)
                    return fakeJobrunner.signalJob(job_id, type)

                await this.call(`/api/jobs/${job_id}/signal/${type}`,
                    {
                        method: "POST",
                    }
                )
            },
            async getJobLog(job_id, abortsignal) {
                if (TESTMODE)
                    return fakeJobrunner.getJobLog(job_id, abortsignal)

                return await this.call(`/api/jobs/${job_id}/log`, {}, false, abortsignal);
            },
            async startBackport(change_urls) {
                if (TESTMODE)
                    return fakeJobrunner.createJob(["scap", "backport"].concat(change_urls))

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
        },
    }
)

export default useAuthStore
