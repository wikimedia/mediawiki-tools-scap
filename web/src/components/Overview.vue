<template>
    <div id="jobrunner-status">{{ jobrunnerStatus }}</div>

    <Backport v-if="idle"/>
    <Train v-if="idle"/>
    <Interaction v-if="interaction" :interaction="this.interaction"/>
    <JobHistory @rowClicked="jobHistoryRowClicked"/>
</template>

<script>
import useApi from '../api';

import Backport from './Backport.vue';
import JobHistory from './JobHistory.vue';
import JobViewer from './JobViewer.vue';
import Interaction from './Interaction.vue';
import Train from './Train.vue';

export default {
    name: 'Overview',
    components: {
        Backport,
        JobHistory,
        JobViewer,
        Interaction,
        Train
    },
    data() {
        return {
            api: null,
            idle: false,
            interaction: null,
            intervalTimer: null,
            jobrunnerStatus: "Jobrunner Status: Unknown"
        }
    },
    mounted() {
        this.api = useApi()
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
                const res = await this.api.getJobrunnerStatus()
                const job_id = res.job_id;

                this.jobrunnerStatus = `Jobrunner Status: ${res.status}`;
                this.idle = res.status == "idle";
                this.interaction = job_id ? await this.api.getJobInteraction(job_id) : null;
            } catch (error) {
                if (!this.api.isAuthenticated) {
                    this.$router.push("/login")
                } else {
                    console.error(`updateJobrunnerStatus caught: ${error.message}`);
                }
            }
        },
        jobHistoryRowClicked(job_id) {
            this.$router.push(`/jobs/${job_id}`)
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
