import { ref, computed, onMounted, onUnmounted } from 'vue';
import useApi from './api';
import { Poller } from './poller';

const status = ref(null);
const idle = computed(() => status.value?.status === "idle");

const poller = new Poller(status, async () => await useApi().getJobrunnerStatus());

export default () => {
	onMounted(async () => await poller.start());
	onUnmounted(async () => await poller.stop());

	return { status, idle };
};
