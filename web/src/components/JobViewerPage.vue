<template>
    <div>
        <div v-if="job">
            <p>Queued by: {{ job.user }} at {{ job.queued_at }}</p>
            <p>Command: {{ job.command_decoded }}</p>
            <p v-if="summary">{{ summary }}</p>
            <p>Status: {{ job.status }}</p>
        </div> 

        <Interaction v-if="interaction" :interaction="this.interaction"/>

        <CdxButton @click="$router.push('/')">Close Log</CdxButton>
        <div id="signal-buttons" v-if="jobRunning">
            <CdxButton id="interrupt" @click="clickSignalButton">Interrupt Job</CdxButton>
            <CdxButton id="kill" @click="clickSignalButton">Kill Job (not recommended)</CdxButton>
        </div>
        <div id="terminal" ref="terminal"></div>
    </div>
</template>

<script>
import "@xterm/xterm/css/xterm.css";
import { Terminal } from '@xterm/xterm';
import { FitAddon } from "@xterm/addon-fit";
import useApi from '../api';
import Interaction from './Interaction.vue'
import { CdxButton } from '@wikimedia/codex';

const fitAddon = new FitAddon();

export default {
    components: {
        CdxButton,
        Interaction
    },
    data() {
        return {
            api: null,
            job: null,
            term: null,
            jobRunning: false,
            monitorInterval: null,
            summary: null,
            interaction: null,
        }
    },
    mounted() {
        this.api = useApi()
        this.job_id = this.$route.params.job_id

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
            const response = await this.api.getJobLog(this.job_id, this.abortcontroller.signal);
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
            const jobinfo = await this.api.getJobInfo(this.job_id)
            const job = jobinfo.job

            job.command_decoded = JSON.parse(job.command).join(' ')
            this.job = job
            this.summary = this.generateSummary(job)
            this.jobRunning = (job.started_at && !job.finished_at)
            this.interaction = await this.api.getJobInteraction(this.job_id)

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
            this.api.signalJob(this.job_id, event.target.id)
        }
    }
}
</script>

<style scoped>
#terminal {
    margin: 5px;
}
</style>