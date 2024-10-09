const API_BASE_URL = "http://localhost:8001"

function makeApiUrl(url) {
    return new URL(url, API_BASE_URL);
}

import { defineStore } from 'pinia'

const authStore = defineStore('spiderpig-auth',
    {
        state() {
            return {
                token: localStorage.getItem('spiderpig-auth-token') || null,
                authFailing: false,
            }
        },
        actions: {
            set(token) {
                this.token = token
                localStorage.setItem('spiderpig-auth-token', token);
            },
            logout() {
                this.token = null
                localStorage.removeItem('spiderpig-auth-token')
            }
        },
        getters: {
            isAuthenticated(state) { return state.token !== null }
        },
    }
)

function isAuthenticated() {
    const auth = authStore()

    return auth.isAuthenticated && !auth.authFailing
}

async function apiCall(url, options, decodeJson = true, signal = null) {
    let localOptions = {
        ...options,
        signal
    }

    const auth = authStore()

    if (isAuthenticated()) {
        localOptions.headers = {
            "Authorization": `Bearer ${auth.token}`
        }
    }

    const response = await fetch(makeApiUrl(url), localOptions)
    auth.authFailing = (response.status == 401)

    if (!response.ok) {
        throw new Error(`Response status: ${response.status}`);
    }

    if (decodeJson) {
        return await response.json();
    } else {
        return response;
    }
}

async function login(username, password) {
    const auth = authStore()

    const formData = new FormData()
    formData.append("username", username)
    formData.append("password", password)

    const response = await fetch(makeApiUrl("/api/login"),
        {
            method: "POST",
            body: formData,
        }
    )
    auth.authFailing = (response.status == 401)

    if (!response.ok) {
        if (response.status === 401) {
            return response
        }
        throw new Error(`Response status: ${response.status}`);
    }   
        
    const res = await response.json();

    auth.set(res.token)
    return true
}

function logout() {
    authStore().logout()
}

async function getJobrunnerStatus() {
    return await apiCall("/api/jobrunner/status");
}

async function getLastNJobs(last=null) {
    const url = new URL("/api/jobs", API_BASE_URL)
    if (last !== null) {
        url.searchParams.append("last", last)
    }
    return await apiCall(url)
}

async function getJobInfo(job_id) {
    return await apiCall(`/api/jobs/${job_id}`);
}

async function getJobInteraction(job_id) {
    const job_info = await getJobInfo(job_id);
    return job_info.pending_interaction;
}

async function respondInteraction(job_id, interaction_id, response) {
    await apiCall(
        `/api/jobs/${job_id}/interact/${interaction_id}`,
        {
            method: "POST",
            body: response,
        }
    );
}

async function signalJob(job_id, type) {
    await apiCall(`/api/jobs/${job_id}/signal/${type}`,
        {
            method: "POST",
        }
    )
}

async function getJobLog(job_id, signal) {
    return await apiCall(`/api/jobs/${job_id}/log`, {}, false, signal);
}

async function startBackport(change_urls) {
    const url = makeApiUrl("/api/jobs/backport")

    for (const change_url of change_urls) {
        url.searchParams.append("change_url", change_url)
    }

    await apiCall(url,
        {
            method: "POST",
        }
    );
}

export { isAuthenticated, login, logout, getJobrunnerStatus, getLastNJobs, getJobInfo, signalJob, respondInteraction, getJobInteraction, getJobLog, startBackport }
