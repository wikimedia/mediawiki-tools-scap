<template>
	<sp-job-page title="MediaWiki Backport">
		<template #new-job="{ idle }">
			<sp-backport :idle="idle" :initial-change-numbers="backportChangeNumbers" />
		</template>
	</sp-job-page>
</template>

<script type="ts">
import { defineComponent, ref } from 'vue';
import { useRouter, useRoute } from 'vue-router';
import SpJobPage from './JobPage.vue';
import SpBackport from './Backport.vue';

export default defineComponent( {
	name: 'BackportPage',
	components: {
		SpJobPage,
		SpBackport
	},
	setup() {
		const backportChangeNumbers = ref( [] );

		const router = useRouter();
		const route = useRoute();

		let backportRequests = route.query.backport;
		if ( typeof backportRequests === 'string' ) {
			backportRequests = [ backportRequests ];
		}

		if ( backportRequests ) {
			backportChangeNumbers.value = backportRequests;
			// Remove the backport query parameter from the URL.
			router.replace( {
				query: {}
			} );
		}

		return { backportChangeNumbers };
	}
} );
</script>
