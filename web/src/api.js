import { defineStore } from 'pinia'
import { useLocalStorage } from "@vueuse/core"
import fakeJobOutput from './fakejoboutput'

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
                fakeJobrunnerInterval: null,
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
                    if (username == "dancy" && password == "123") {
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
                if (TESTMODE) {
                    this.startFakeJobrunner()

                    if (this.testmodeData.currentJob) {
                        return {
                            status: `Running job ${this.testmodeData.currentJob.id}`,
                            job_id: this.testmodeData.currentJob.id
                        }
                    } else {
                        return {
                            status: "idle",
                            job_id: null,
                        }
                    }
                }

                return await this.call("/api/jobrunner/status");
            },
            async getLastNJobs(last = null) {
                if (TESTMODE) {
                    return {
                        jobs: this.testmodeData.jobs
                    }
                }

                const url = new URL("/api/jobs", API_BASE_URL)
                if (last !== null) {
                    url.searchParams.append("last", last)
                }
                return await this.call(url)
            },
            getJobById(job_id) {
                // Returns a job object, or undefined if not found
                return this.testmodeData.jobs.find((job) => job.id == job_id)
            },
            async getJobInfo(job_id) {
                if (TESTMODE) {
                    const job = this.getJobById(job_id)

                    return {
                        "job": job,
                        "pending_interaction": null
                    }
                }

                return await this.call(`/api/jobs/${job_id}`);
            },
            async getJobInteraction(job_id) {
                const job_info = await this.getJobInfo(job_id);
                return job_info.pending_interaction;
            },
            async respondInteraction(job_id, interaction_id, response) {
                await this.call(
                    `/api/jobs/${job_id}/interact/${interaction_id}`,
                    {
                        method: "POST",
                        body: response,
                    }
                );
            },
            async signalJob(job_id, type) {
                if (TESTMODE) {
                    // FIXME: add something to the job's log

                    const job = this.getJobById(job_id)
                    if (!job) {
                        return
                    }

                    this.terminateFakeJob(job, `Terminated by ${type} signal`, null)
                    return
                }


                await this.call(`/api/jobs/${job_id}/signal/${type}`,
                    {
                        method: "POST",
                    }
                )
            },
            fakeLogGenerator: async function* () {

            },
            async getJobLog(job_id, signal) {
                if (TESTMODE) {
                    const job = this.getJobById(job_id)

                    if (!job) {
                        // FIXME: Do something equivalent to a 404 response
                        return
                    }

                    return {
                        body: this.fakeJobOutputIterable()
                    }
                }

                return await this.call(`/api/jobs/${job_id}/log`, {}, false, signal);
            },
            async startBackport(change_urls) {
                if (TESTMODE) {
                    const job = {
                        "id": ++this.testmodeData.lastJobId,
                        "queued_at": new Date().toISOString(),
                        "started_at": null,
                        "finished_at": null,
                        "exit_status": null,
                        "user": "dancy",
                        "command": JSON.stringify(["scap", "backport"].concat(change_urls)),
                        "status": null,
                    }

                    this.testmodeData.jobs.unshift(job)
                    return
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
            startFakeJobrunner() {
                if (!this.fakeJobrunnerInterval) {
                    this.fakeJobrunnerInterval = setInterval(this.fakeJobrunner, 2000)
                }
            },
            stopFakeJobrunner() {
                if (this.fakeJobrunnerInterval) {
                    clearInterval(this.fakeJobrunnerInterval)
                    this.fakeJobrunnerInterval = null
                }
            },

            async fakeJobrunner() {
                if (!this.testmodeData.currentJob) {
                    const job = this.testmodeData.jobs.find((job) => !job.started_at)

                    if (job) {
                        console.log(`fakeJobrunner: Starting job ${job.id}`)
                        this.startFakeJob(job)
                        this.testmodeData.currentJob = job
                    }
                } else {
                    const job = this.testmodeData.currentJob
                    const elapsed = (new Date() - new Date(job.started_at)) / 1000

                    if (elapsed > 10) {
                        console.log(`fakeJobrunner: Terminating job ${job.id}`)
                        this.terminateFakeJob(job, `Job ${job.id} finished normally`, 0)
                    }
                }
            },
            startFakeJob(job) {
                job.started_at = new Date().toISOString()
                job.testmode_log = []
            },
            terminateFakeJob(job, status, exit_status) {
                job.finished_at = new Date().toISOString()
                job.status = status
                if (this.testmodeData.currentJob === job) {
                    this.testmodeData.currentJob = null
                }
            },
            fakeJobOutputIterable() {
                let index = 0
                let lastTimestamp = null

                return {
                    [Symbol.asyncIterator]() {
                        return {
                            next() {
                                return new Promise((resolve, reject) => {
                                    if (index < fakeJobOutput.length) {
                                        let wait = 0
                                        if (lastTimestamp) {
                                            wait = fakeJobOutput[index].timestamp - lastTimestamp
                                        }
                                        wait *= 1000

                                        setTimeout(() => {
                                            resolve({ value: fakeJobOutput[index].line, done: false })
                                            lastTimestamp = fakeJobOutput[index].timestamp
                                            index++
                                        }, wait)
                                    } else {
                                        resolve({ done: true })
                                    }
                                })
                            }
                        }
                    }
                }
            },
        },
    }
)

export default useAuthStore