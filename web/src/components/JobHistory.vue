<template>
    <h3>Job History</h3>
    <table v-if="loaded" id="job-history-table">
        <tr>
            <th>Id</th>
            <th>Queued</th>
            <th>User</th>
            <th>Command</th>
            <th>Started</th>
            <th>Finished</th>
            <th>Status</th>
        </tr>
        <tr v-for="job in jobs" :key="job.id" :class="{
            failed: Number.isInteger(job.exit_status) && job.exit_status != 0,
            running: job.started_at && !job.finished_at
        }" @click="rowClicked">
            <td>{{ job.id }}</td>
            <td>{{ job.queued_at }}</td>
            <td>{{ job.user }}</td>
            <td>{{ job.command_decoded }}</td>
            <td>{{ job.started_at }}</td>
            <td>{{ job.finished_at_massaged }}</td>
            <td>{{ job.status }}</td>
        </tr>
    </table>
    <div v-else>Loading...</div>
</template>

<script>
import useApi from '../api'

export default {
    emits: ['rowClicked'],
    data() {
        return {
            api: null,
            loaded: false,
            jobs: [],
            intervalTimer: null,
        }
    },
    mounted() {
        this.api = useApi()
        this.intervalTimer = setInterval(this.loadHistory, 1000);
        this.loadHistory();
    },
    unmounted() {
        if (this.intervalTimer) {
            clearInterval(this.intervalTimer);
            this.intervalTimer = null;
        }
    },
    methods: {
        async loadHistory() {
            try {
                const res = await this.api.getLastNJobs(5);
                const jobs = res.jobs;

                for (const job of jobs) {
                    job.command_decoded = JSON.parse(job.command).join(" ");
                    job.finished_at_massaged = job.finished_at
                    if (job.started_at && !job.finished_at) {
                        job.finished_at_massaged = "..Running..."
                    }
                }

                this.jobs = jobs;
                this.loaded = true;
            } catch (error) {
                console.error(error.message);
            }
        },
        rowClicked(e) {
            this.$emit('rowClicked', e.currentTarget.cells[0].innerText);
        }
    }

}
</script>

<style scoped>
#job-history-table {
    border-style: solid;
    border-width: 2px;
}

#job-history-table th {
    background-color: lightgreen;
}

#job-history-table tr {
    line-height: 30px;
    cursor: pointer;
}

#job-history-table tr:nth-child(even) {
    background-color: #f2f2f2;
}

#job-history-table tr.running {
    background-color: lightblue;
}

#job-history-table tr.failed {
    background-color: red;
}

#job-history-table tr:hover {
    background-color: coral;
}

#job-history-table td {
    padding: 5px;
}
</style>