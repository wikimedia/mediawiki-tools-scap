<template>
    <div id="jobrunner-status">{{ jobrunnerStatus }}</div>

    <Backport v-if="idle"/>

    <Train v-if="idle"/>

    <Interaction v-if="interaction" :interaction="this.interaction"/>

    <JobHistory />
</template>

<script>
import useApi from '../api';
import Backport from './Backport.vue';
import JobHistory from './JobHistory.vue';
import JobViewer from './JobViewerPage.vue';
import Interaction from './Interaction.vue';
import Train from './Train.vue';

export default {
    name: 'OverviewPage',
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

                this.jobrunnerStatus = `Jobrunner Status: ${res.status}`;
                this.idle = res.status == "idle";
                this.interaction = res.pending_interaction
            } catch (error) {
                if (!this.api.isAuthenticated) {
                    this.$router.push("/login")
                } else {
                    console.error(`updateJobrunnerStatus caught: ${error.message}`);
                }
            }
        }
    }
}
</script>

<style scoped>
</style>
