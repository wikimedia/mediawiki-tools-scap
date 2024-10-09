<template>
    <div id="jobrunner-status">{{ jobrunnerStatus }}</div>

    <Backport v-if="idle"/>
    <Interaction v-if="interaction" :interaction="this.interaction"/>
    <JobHistory v-if="!viewingJob" @rowClicked="jobHistoryRowClicked"/>
    <JobViewer v-if="viewingJob" :job_id="this.viewingJob" @closeJobViewer="closeJobViewer"/>
</template>

<script>
import { getJobrunnerStatus, getJobInteraction, isAuthenticated } from '../common.js'
import Backport from './Backport.vue';
import JobHistory from './JobHistory.vue';
import JobViewer from './JobViewer.vue';
import Interaction from './Interaction.vue';


export default {
    name: 'Overview',
    components: {
        Backport,
        JobHistory,
        JobViewer,
        Interaction
    },
    data() {
        return {
            idle: false,
            viewingJob: null,
            interaction: null,
            intervalTimer: null,
            jobrunnerStatus: "Jobrunner Status: Unknown"
        }
    },
    mounted() {
        this.intervalTimer = setInterval(this.updateJobrunnerStatus, 1000);
        this.updateJobrunnerStatus();
    },
    unmounted() {
        if (this.intervalTimer) {
            clearInterval(this.intervalTimer);
            this.intervalTimer = null;
        }
    },
    methods: {
        async updateJobrunnerStatus() {
            try {
                const res = await getJobrunnerStatus()
                const job_id = res.job_id;

                this.jobrunnerStatus = `Jobrunner Status: ${res.status}`;
                this.idle = res.status == "idle";
                this.interaction = job_id ? await getJobInteraction(job_id) : null;
            } catch (error) {
                if (!isAuthenticated()) {
                    this.$router.push("/login")
                } else {
                    console.error(`updateJobrunnerStatus caught: ${error.message}`);
                }
            }
        },
        jobHistoryRowClicked(job_id) {
            this.viewingJob = job_id;
        },
        closeJobViewer() {
            this.viewingJob = null;
        },
    }
}
</script>

<style scoped>

#jobrunner-status {
    display: table;
    border-style: solid;
    border-radius: 10px;
    border-width: 1px;
    padding: 5px;
    margin: 5px;
}
</style>
