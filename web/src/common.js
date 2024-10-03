const API_BASE_URL = "http://localhost:8001"

function makeApiUrl(url) {
    return new URL(url, API_BASE_URL);
}

async function apiCall(url, options, decodeJson = true, signal = null) {
    const response = await fetch(
        makeApiUrl(url),
        {
            ...options,
            headers: {
                "Authorization": "Bearer dancy",
            },
            signal
        }
    )
    if (!response.ok) {
        throw new Error(`Response status: ${response.status}`);
    }

    if (decodeJson) {
        return await response.json();
    } else {
        return response;
    }
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

export { getJobrunnerStatus, getLastNJobs, getJobInfo, signalJob, respondInteraction, getJobInteraction, getJobLog, startBackport }
