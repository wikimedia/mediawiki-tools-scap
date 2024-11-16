<template>
	<cdx-field id="backport">
		<template #label>
			Backport patches
		</template>

		<template #description>
			To run scap backport, enter one or more Gerrit change id numbers
			separated by spaces.
		</template>

		<div id="backport__input">
			<cdx-multiselect-lookup
				id="backport__input__multiselect"
				v-model:input-chips="changeIds"
				v-model:selected="selection"
				:menu-items="menuItems"
				:menu-config="menuConfig"
				placeholder="Enter change numbers"
				aria-label="Search for one or more Gerrit patches"
				@input="onInput"
			>
				<template #no-results>
					No results found.
				</template>
			</cdx-multiselect-lookup>

			<cdx-button
				:disabled="buttonDisabled"
				@click="startBackport"
			>
				Start Backport
			</cdx-button>
		</div>
	</cdx-field>
</template>

<script lang="ts">
import { ref, computed } from 'vue';
import { CdxButton, CdxField, CdxMultiselectLookup } from '@wikimedia/codex';
import useApi from '../api';

export default {
	name: 'SpBackport',
	components: {
		CdxButton,
		CdxField,
		CdxMultiselectLookup
	},

	setup() {
		const api = useApi();
		const changeIds = ref( [] );
		const selection = ref( [] );
		const menuItems = ref( [] );

		const menuConfig = {
			boldLabel: true,
			visibleItemLimit: 10
		};

		const buttonDisabled = computed( () => {
			if ( changeIds.value.length > 0 ) {
				return false;
			} else {
				return true;
			}
		} );

		async function startBackport() {
			await api.startBackport( changeIds.value.map( ( id ) => id.value ) );
			changeIds.value = [];
		}

		async function fetchResults( changeId ) {
			return await api.searchPatch(
				`change:"${ changeId }"`,
				10
			);
		}

		async function onInput( value ) {
			if ( !value ) {
				menuItems.value = [];
				return;
			}

			const data = await fetchResults( value );

			const results = data.map( ( item ) => {
				// eslint-disable-next-line no-underscore-dangle
				const changeNumber = String( item._number );

				return {
					label: changeNumber, // the Gerrit patch number, ex. 1084887
					description: item.subject, // The commit subject line
					value: changeNumber // the Gerrit patch number, ex. 1084887
				};
			} );

			menuItems.value = results;
		}

		return {
			buttonDisabled,
			changeIds,
			selection,
			menuItems,
			menuConfig,
			startBackport,
			onInput
		};
	}
};
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';

#backport {
	margin-bottom: @spacing-150;

	&__input {
		display: flex;

		&__multiselect {
			width: 65%;
		}
	}

	.change-id-input {
		flex: 1 1 auto;
	}
}

</style>
