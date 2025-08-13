<template>
	<cdx-info-chip>
		{{ total }}
	</cdx-info-chip>
</template>

<script>
import { defineComponent, ref, onMounted, onUnmounted } from 'vue';
import { CdxInfoChip } from '@wikimedia/codex';
import useApi from '../api';

export default defineComponent( {
	name: 'ErrorLogsCount',
	components: { CdxInfoChip },
	setup() {
		const api = useApi();
		const total = ref( '⌚' );
		const INTERVAL = 15000;
		let intervalTimerTotal = null;

		const populateTotal = async () => {
			try {
				const totalResp = await api.getLogsTotal();
				total.value = totalResp.total;
			} catch ( error ) {
				// eslint-disable-next-line no-console
				console.error( 'Error fetching logs total:', error );
				total.value = '⌚';
			}
		};

		onMounted( async () => {
			// Fire off initial population without waiting.
			populateTotal();
			intervalTimerTotal = window.setInterval( populateTotal, INTERVAL );
		} );

		onUnmounted( () => {
			if ( intervalTimerTotal ) {
				clearInterval( intervalTimerTotal );
				intervalTimerTotal = null;
			}
		} );

		return {
			total
		};
	}
} );
</script>
