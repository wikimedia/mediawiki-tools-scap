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
			<div class="navigation__end___first">
				<div
					class="cdx-progress-bar"
					role="progressbar"
					aria-label="Faking progress"
				>
					<div class="cdx-progress-bar__bar spiderpig-progress-bar__bar" />
				</div>
				<ul>
					<li>
						<a href="https://logstash.wikimedia.org/app/dashboards#/view/mediawiki-errors" target="_blank">
							Logstash<cdx-icon :icon="cdxIconLinkExternal" />
						</a>
					</li>
				</ul>
			</div>

			<user-menu />
		</div>
	</nav>
</template>

<script lang="ts">
import { onMounted } from 'vue';
import UserMenu from './UserMenu.vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconLinkExternal } from '@wikimedia/codex-icons';

export default {
	name: 'SpNavigation',
	components: {
		UserMenu,
		CdxIcon
	},

	setup() {
		onMounted( () => {
			let percentage = 0;
			const progressBar = document.querySelector( '.spiderpig-progress-bar__bar' );
			window.thingy = progressBar;

			function doProgress() {
				if ( percentage > 99 ) {
					percentage = 0;
					if ( confirm( 'Are you not entertained!?' ) ) {
						return;
					}
				}
				percentage += 10;
				progressBar.style.width = percentage + '%';
				setTimeout( doProgress, 100 );
			}

			setTimeout( doProgress, 100 );
		} );
		return {
			cdxIconLinkExternal
		};
	}
};
</script>

<style lang="less" scoped>
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.navigation {
	display: flex;
	align-items: center;
	color: @color-base;

	&__start {
		flex: 0 0 3rem;

		img {
			width: @size-300;
			height: @size-300;
		}
	}

	&__center {
		flex: 1 1 auto;
		display: flex;
		flex-direction: column;
		font-family: Montserrat, Avenir, Corbel, 'URW Gothic', source-sans-pro, sans-serif;
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
		display: flex;
		&___first {
			.spiderpig-progress-bar__bar {
				animation: none;
			}
			ul {
				list-style-type: none;
				li {
					a {
						display: block;
						padding: 0.25rem 0.5rem;
						.cdx-mixin-link();
						.cdx-icon {
							color: inherit;
						}
					}
				}
			}
		}
	}
}
</style>
