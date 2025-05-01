<template>
	<sp-new-job-page title="MediaWiki Backport">
		<template #new-job="{ idle }">
			<sp-backport :idle="idle" :initial-change-numbers="backportChangeNumbers" />
		</template>
	</sp-new-job-page>
</template>

<script type="ts">
import { defineComponent, ref } from 'vue';
import { useRouter, useRoute } from 'vue-router';
import SpNewJobPage from './NewJobPage.vue';
import SpBackport from './Backport.vue';

export default defineComponent( {
	name: 'BackportPage',
	components: {
		SpNewJobPage,
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
