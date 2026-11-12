import { computed } from 'vue';
import useEvents, { jobrunnerStatus as status } from './events';

// The jobrunner runs one job at a time in each queue.  Keep these names in
// sync with MEDIAWIKI_QUEUE and SERVICE_QUEUE_PREFIX in scap/spiderpig/model.py.
export const MEDIAWIKI_QUEUE = 'mediawiki';
export const serviceQueue = ( service: string ) => `service:${ service }`;

const runningJobs = computed(() => status.value?.running_jobs ?? []);
const busyQueues = computed(() => status.value?.busy_queues ?? []);
const idle = computed(() => status.value?.status === "idle");

// True while a job is running in the named queue.  A job running in another queue does not
// hold up work here.
function queueIsBusy( queue: string ): boolean {
	return busyQueues.value.includes( queue );
}

const mediawikiIsBusy = computed( () => queueIsBusy( MEDIAWIKI_QUEUE ) );

// The running job of the given type, if the jobrunner runs one.
function runningJobOfType( type: string ) {
	return runningJobs.value.find( ( job ) => job.type === type ) ?? null;
}

export default () => {
	useEvents();

	return {
		status,
		idle,
		runningJobs,
		busyQueues,
		queueIsBusy,
		mediawikiIsBusy,
		runningJobOfType
	};
};
