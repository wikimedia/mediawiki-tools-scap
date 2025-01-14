<template>
	<nav class="navigation">
		<div class="navigation__start">
			<img src="../assets/spiderpig.png">
		</div>

		<div class="navigation__center">
			<h1>
				SPIDER PIG
			</h1>
			<h1>
				DEPLOYMENT
			</h1>
		</div>

		<div class="navigation__end">
			<cdx-button
				v-if="isLoggedIn"
				action="progressive"
				weight="quiet"
				@click="logout"
			>
				Logout
			</cdx-button>
		</div>
	</nav>
</template>

<script lang="ts">
import { computed } from 'vue';
import { useRouter } from 'vue-router';
import { CdxButton } from '@wikimedia/codex';
import useApi from '../api';

export default {
	name: 'SpNavigation',
	components: {
		CdxButton
	},

	setup() {
		// Pinia store and router.
		const router = useRouter();
		const api = useApi();
		const isLoggedIn = computed( () => api.isAuthenticated );

		function logout() {
			api.logout();
			router.push( '/login' );
		}

		return {
			isLoggedIn,
			logout
		};
	}
};
</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

.navigation {
	display: flex;
	align-items: center;
	color: @color-base;

}

.navigation__start {
	flex: 0 0 3rems;

	img {
		width: @size-300;
		height: @size-300;
	}
}

.navigation__center {
	flex: 1 1 auto;
	display: flex;
	flex-direction: column;
	font-family: Montserrat;
	font-size: @font-size-medium;
	font-style: normal;
	font-weight: 800;
	line-height: normal;
	letter-spacing: 0.4px;
	padding-left: @spacing-25;
	align-self: stretch;

	h1:last-child {
		font-size: @font-size-x-large;
		font-weight: @font-weight-semi-bold;
	}
}

.navigation__end {
	flex: 0 0 100px;
}
</style>
