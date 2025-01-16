<template>
	<div class="change-details">
		<div class="change-details__grid">
			<div class="change-details__grid__item">
				<div class="change-details__label">
					Change number
				</div>
				<div>
					<a
						class="change-details__link"
						:href="url"
						target="_blank"
					>
						{{ number }}
					</a>
				</div>
			</div>
			<div class="change-details__grid__item">
				<div class="change-details__label">
					Repo
				</div>
				<div>
					<a
						class="change-details__link"
						:href="repoURL"
						target="_blank"
					>
						{{ project }}
					</a>
				</div>
			</div>
			<div class="change-details__grid__item">
				<div class="change-details__label">
					Branch
				</div>
				<div>
					<a
						class="change-details__link"
						:href="branchURL"
						target="_blank"
					>
						{{ branch }}
					</a>
				</div>
			</div>
		</div>
		<div>
			<p class="change-details__subject">
				{{ subject }}
			</p>
			<p class="change-details__commit-msg">
				<pre>
					{{ formattedCommitMsg }}
				</pre>
			</p>
		</div>
		<div
			v-for="( bug, index ) in bugs"
			:key="index"
			class="change-details__bug"
		>
			Bug:
			<a
				class="change-details__link"
				:href="bug.url"
				target="_blank"
			>
				{{ bug.id }}
			</a>
		</div>
		<div class="change-details__change-id">
			Change-Id:
			<a
				class="change-details__link"
				:href="changeIdURL"
				target="_blank"
			>
				{{ changeId }}
			</a>
		</div>
	</div>

</template>

<script lang="ts">
import { defineComponent } from 'vue';

export default defineComponent( {
	name: 'ChangeDetails',

	props: {
		project: {
			type: String,
			required: true
		},

		branch: {
			type: String,
			required: true
		},

		number: {
			type: Number,
			required: true
		},

		url: {
			type: String,
			required: true
		},

		repoURL: {
			type: String,
			required: true
		},

		branchURL: {
			type: String,
			default: null
		},

		changeIdURL: {
			type: String,
			default: null
		},

		subject: {
			type: String,
			required: true
		},

		formattedCommitMsg: {
			type: String,
			required: true
		},

		bugs: {
			type: Array,
			default: () => []
		},

		changeId: {
			type: String,
			required: true
		}
	}
} );
</script>

<style lang="less">
@import '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.change-details {
	&__grid {
		@media screen and ( min-width: @min-width-breakpoint-tablet ) {
			display: grid;
			grid-template-columns: repeat( 4, 1fr );
			gap: @spacing-12;
			grid-gap: @spacing-12;
			padding-bottom: @spacing-100;
		}
	}

	&__label {
		font-weight: @font-weight-semi-bold;
	}

	&__link {
		.cdx-mixin-link();
	}

	&__subject,
	&__bug,
	&__change-id {
		font-family: monospace;
		font-size: @font-size-small;
	}

	&__commit-msg pre {
		font-family: monospace;
		font-size: @font-size-small;
	}
}
</style>
