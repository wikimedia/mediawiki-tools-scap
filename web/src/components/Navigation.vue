<template>
	<nav class="navigation">
		<div class="navigation__start">
			<img src="../assets/spiderpig.png">
		</div>

		<div class="navigation__center">
			<h1>{{ title }}</h1>
		</div>

		<div class="navigation__end">
			<cdx-button v-if="isLoggedIn" @click="logout">
				Logout
			</cdx-button>
		</div>
	</nav>
</template>

<script lang="ts">
import { computed } from 'vue';
import { useRouter, useRoute } from 'vue-router';
import { CdxButton } from '@wikimedia/codex';
import useApi from '../api';

export default {
	name: 'SpNavigation',
	components: {
		CdxButton
	},

	setup() {
		const router = useRouter();
		const route = useRoute();
		const api = useApi();

		/**
		 * Show a title based on the route "meta" information and the Job ID,
		 * if available
		 */
		const title = computed( () => {
			if ( route.params.job_id && route.meta.title ) {
				return `${ route.meta.title }${ route.params.job_id }`;
			} else {
				return route.meta.title;
			}
		} );

		const isLoggedIn = computed( () => api.isAuthenticated );

		function logout() {
			api.logout();
			router.push( '/login' );
		}

		return {
			title,
			isLoggedIn,
			logout
		};
	}
};
</script>

<style scoped>
.navigation {
	display: flex;
	align-items: center;
}

.navigation__start {
	flex: 0 0 3rems;
}

.navigation__center {
	flex: 1 1 auto;
}

.navigation__end {
	flex: 0 0 100px;
}
</style>
