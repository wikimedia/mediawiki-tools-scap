<template>
	<nav class="navigation">
		<div class="navigation__start">
			<router-link to="/" tabindex="-1">
				<img src="../assets/spiderpig.png" alt="Spiderpig logo">
			</router-link>
		</div>

		<div class="navigation__center">
			<router-link to="/">
				<h1>
					SPIDER PIG
					<span>
						DEPLOYMENT
					</span>
				</h1>
			</router-link>
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

	&__start {
		flex: 0 0 3rems;

		img {
			width: @size-300;
			height: @size-300;
		}
	}

	&__center {
		flex: 1 1 auto;
		display: flex;
		flex-direction: column;
		font-family: Montserrat, serif;
		font-size: @font-size-medium;
		font-style: normal;
		font-weight: @font-weight-bold;
		line-height: normal;
		letter-spacing: 0.4px;
		padding-left: @spacing-50;
		align-self: stretch;

		a {
			width: fit-content;
			color: inherit;
			border-radius: @border-radius-base;
			text-decoration: none;

			h1 {
				color: inherit;
				border-radius: @border-radius-base;
				text-decoration: none;

				span {
					display: block;
					font-size: @font-size-x-large;
					font-weight: @font-weight-semi-bold;
				}
			}
		}
	}

	&__end {
		flex: 0 0 100px;
	}
}
</style>
