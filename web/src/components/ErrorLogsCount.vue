<template>
	<cdx-info-chip
		:status="staleErrorPollTime === null ? 'notice' : 'warning'"
		:title="staleWarning"
	>
		{{ errorTotal ?? '⌚' }}
	</cdx-info-chip>
</template>

<script>
import { computed, defineComponent } from 'vue';
import { CdxInfoChip } from '@wikimedia/codex';
import useEvents from '../events';
import { useAgeFormatter } from '../time';

export default defineComponent( {
	name: 'ErrorLogsCount',
	components: { CdxInfoChip },
	setup() {
		const { errorTotal, staleErrorPollTime } = useEvents();
		const formatAge = useAgeFormatter();

		const staleWarning = computed( () => ( staleErrorPollTime.value === null ? undefined :
			`The last successful logstash poll was ${ formatAge( staleErrorPollTime.value ) }` ) );

		return {
			errorTotal,
			staleErrorPollTime,
			staleWarning
		};
	}
} );
</script>
