<template>
    <div id="job-history">
        <cdx-table
            caption="Job History"
            :columns="columns"
            :data="jobs"
        >
            <template #item-id="{ item, row }">
                <router-link :to="{ name: 'job', params: { job_id: row.id } }">
                    {{ item }}
                </router-link>
            </template>

            <template #item-command_decoded="{ item, row }">
                {{ item }}
            </template>

            <template #item-user="{ item, row }">
                {{ item }}
            </template>

            <template #item-queued_at="{ item, row }">
                {{ getFormattedDate( item ) }}
            </template>

            <template #item-started_at="{ item, row }">
                {{ getFormattedDate( item ) }}
            </template>

            <template #item-finished_at_message="{ item, row }">
                {{ getFormattedDate( item ) }}
            </template>

            <template #item-status="{ item, row }">
                <cdx-message :inline="true" :type="getStatusType( row.exit_status )">
                    {{ item }}
                </cdx-message>
            </template>


            <template #empty-state>
                No job data to display.
            </template>
        </cdx-table>
    </div>
</template>

<script>
import { ref, onMounted, onUnmounted } from 'vue';
import { CdxTable, CdxMessage } from '@wikimedia/codex';
import useApi from '../api';

const INTERVAL = 1000;
const JOB_RUNNING = '..Running...';

export default {
    name: 'JobHistory',
    components: {
        CdxTable,
        CdxMessage
    },
    emits: [
        'rowClicked'
    ],
    setup( props, { emit } ) {
        const api = useApi();
        const loaded = ref( false );
        const jobs = ref( [] );
        const intervalTimer = ref( null );

        const columns = [
            { id: 'id', label: 'Id' },
            { id: 'command_decoded', label: 'Command', width: '35%' },
            { id: 'user', label: 'User' },
            { id: 'started_at', label: 'Started' },
            { id: 'finished_at_message', label: 'Finished' },
            { id: 'status', label: 'Status', width: '15%' }
        ];

        async function loadHistory() {
            try {
                const apiResponse = await api.getLastNJobs( 5 );
                const apiJobs = apiResponse.jobs;

                for ( const job of apiJobs ) {
                    job.command_decoded = JSON.parse( job.command ).join( " " );
                    job.finished_at_message = job.finished_at;

                    if ( job.started_at && !job.finished_at ) {
                        job.finished_at_message = JOB_RUNNING;
                    }
                }

                jobs.value = apiJobs;
                loaded.value = true;
            } catch ( error ) {
                console.error( error.message );
            }
        }

        function getFormattedDate( dateString ) {
            // Handle the job-in-progress message by bailing early
            if ( dateString === JOB_RUNNING ) {
                return;
            }

            const date = new Date( dateString );
            return date.toUTCString();
        }

        function getStatusType( exit ) {
            const statusMap = {
                0: 'success',
                1: 'error'
            }

            if ( exit ) {
                return statusMap[ exit ];
            }
        }

        onMounted( () => {
            loadHistory();
            intervalTimer.value = setInterval( loadHistory, INTERVAL );
        } );

        onUnmounted( () => {
            if ( intervalTimer.value ) {
                clearInterval( intervalTimer.value );
                intervalTimer.value = null;
            }
        } );

        return {
            columns,
            jobs,
            loaded,
            getFormattedDate,
            getStatusType
        };
    }
};
</script>