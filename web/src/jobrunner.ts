import { ref, computed, onMounted, onUnmounted } from 'vue';
import useApi from './api';
import { Poller } from './poller';

// The jobrunner runs one job at a time in each queue.  Keep these names in
// sync with MEDIAWIKI_QUEUE and SERVICE_QUEUE_PREFIX in scap/spiderpig/model.py.
export const MEDIAWIKI_QUEUE = 'mediawiki';
export const serviceQueue = ( service: string ) => `service:${ service }`;

const status = ref(null);
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

const poller = new Poller(status, async () => await useApi().getJobrunnerStatus());

export default () => {
	onMounted(async () => await poller.start());
	onUnmounted(async () => await poller.stop());

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
