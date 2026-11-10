<template>
	<cdx-info-chip>
		{{ errorTotal ?? '⌚' }}
	</cdx-info-chip>
</template>

<script>
import { defineComponent } from 'vue';
import { CdxInfoChip } from '@wikimedia/codex';
import useEvents from '../events';

export default defineComponent( {
	name: 'ErrorLogsCount',
	components: { CdxInfoChip },
	setup() {
		const { errorTotal } = useEvents();

		return {
			errorTotal
		};
	}
} );
</script>
