<template>
    <div>
        <h1>Welcome to job #{{ job_id }}</h1>
        <div v-if="job">
            <p>Queued by: {{ job.user }} at {{ job.queued_at }}</p>
            <p>Command: {{ job.command }}</p>
            <p v-if="summary">{{ summary }}</p>
        </div> 
        <button @click="$emit('closeJobViewer')">Close Log</button>
        <div id="signal-buttons" v-if="jobRunning">
            <button id="interrupt" @click="clickSignalButton">Interrupt Job</button>
            <button id="kill" @click="clickSignalButton">Kill Job</button>
        </div>
        <div id="terminal" ref="terminal"></div>
    </div>
</template>

<script>
import "@xterm/xterm/css/xterm.css";
import { Terminal } from '@xterm/xterm';
import { FitAddon } from "@xterm/addon-fit";
import { getJobInfo, signalJob, getJobLog } from '../common.js'

const fitAddon = new FitAddon();

export default {
    props: {
        'job_id': {
            type: String,
            required: true,
        }
    },
    emits: ['closeJobViewer'],
    data() {
        return {
            job: null,
            term: null,
            jobRunning: false,
            monitorInterval: null,
            summary: null,
        }
    },
    mounted() {
        const term = new Terminal({
            convertEol: true
        });
        term.loadAddon(fitAddon);
        term.open(this.$refs.terminal);
        fitAddon.fit();

        window.addEventListener('resize', this.resizeTerminal);
        this.term = term;
        this.abortcontroller = new AbortController();
        this.populateTerminal();

        this.monitorInterval = setInterval(this.monitorJob, 1000);
        this.monitorJob();
    },
    unmounted() {
        if (this.abortcontroller) {
            this.abortcontroller.abort();
            this.abortcontroller = null;
        }
        this.stopMonitor();
        window.removeEventListener('resize', this.resizeTerminal);
    },
    methods: {
        async populateTerminal() {
            const response = await getJobLog(this.job_id, this.abortcontroller.signal);
            try {
                for await (const chunk of response.body) {
                    this.term.write(chunk)
                }
            } catch (err) {
                if (err.name !== 'AbortError') {
                    console.log(`populateTerminal caught ${err}`)
                }
            }
        },
        resizeTerminal() {
            fitAddon.fit();
        },
        stopMonitor() {
            if (this.monitorInterval) {
                clearInterval(this.monitorInterval);
                this.monitorInterval = null;
            }
        },
        async monitorJob() {
            const jobinfo = await getJobInfo(this.job_id)
            const job = jobinfo.job

            job.command = JSON.parse(job.command).join(' ')
            this.job = job
            this.summary = this.generateSummary(job)
            this.jobRunning = (job.started_at && !job.finished_at)
            if (!this.jobRunning) {
                this.stopMonitor()
            }

        },
        generateSummary(job) {
            if (!job.started_at) {
                return "Job has not started yet"
            }

            let summary = `Job started at ${job.started_at}`
            if (job.finished_at) {
                summary += ` and finished at ${job.finished_at} with exit status ${job.exit_status}`
            } else {
                summary += " and is still running"
            }
            return summary
        },
        clickSignalButton(event) {
            signalJob(this.job_id, event.target.id)
        }
    }
}
</script>

<style scoped>
#terminal {
    margin: 5px;
}
</style>