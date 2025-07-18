<template>
	<v-card flat>
		<template #text>
			<v-text-field
				v-model="search"
				prepend-inner-icon="mdi-magnify"
				hide-details
				single-line
				label="Filter"
			/>
		</template>

		<v-data-table
			:headers="headers"
			:items="items"
			:search="search"
			:filter-keys="[ 'versions', 'message.message' ]"
			item-value="message"
		>
			<template #item.message="{ value }">
				<a
					:href="value.link"
					class="cdx-link"
					target="_blank"
				>
					{{ value.message }}
					<cdx-icon :icon="cdxIconLinkExternal" />
				</a>
			</template>
		</v-data-table>
	</v-card>
</template>

<script>
import { onMounted, onUnmounted, defineComponent, ref } from 'vue';
import { CdxIcon } from '@wikimedia/codex';
import { cdxIconLinkExternal } from '@wikimedia/codex-icons';
const search = ref( '' );

import useApi from '../api';

export default defineComponent( {
	name: 'SpLogs',
	components: {
		CdxIcon
	},
	setup() {
		const customMessageSort = ( a, b ) => a.message.localeCompare( b.message );

		const headers = [
			{ title: 'Count', align: 'start', key: 'count' },
			{ title: 'Versions', align: 'start', key: 'versions' },
			{ title: 'Message', align: 'start', key: 'message', sort: customMessageSort }
		];

		const items = ref();
		const api = useApi();
		const maxMessageLength = 1000;
		const INTERVAL = 15000;
		let intervalTimer = null;

		const createOpenSearchLink = ( errorMessage, fieldName = 'normalized_message', timeRange = '24h' ) => {
			const dashboard = 'https://logstash.wikimedia.org/app/dashboards#/view/mediawiki-errors';
			const timeFrom = `now-${ timeRange }`;
			const errorMessageForURI = encodeURIComponent( errorMessage );

			const filterStr = `(meta:(alias:!n,disabled:!f,key:${ fieldName },negate:!f,` +
				`params:(query:'${ errorMessageForURI }'),type:phrase),` +
				`query:(match_phrase:(${ fieldName }:'${ errorMessageForURI }')))`;

			const globalState = `(filters:!(),refreshInterval:(pause:!t,value:0),time:(from:${ timeFrom },to:now))`;

			const appState = `(columns:!(_source),filters:!(${ filterStr }),index:'logstash-*',` +
				'interval:auto,query:(language:kuery,query:\'\'),sort:!())';

			return `${ dashboard }?_g=${ globalState }&_a=${ appState }`;
		};

		const populateLogs = async () => {
			const resp = await api.getLogs(),
				newData = [];
			for ( const [ key, value ] of Object.entries( resp.log ) ) {
				let message = key;
				const link = createOpenSearchLink( key );
				if ( key.length > maxMessageLength ) {
					message = key.slice( 0, maxMessageLength ) + '...';
				}
				newData.push( {
					count: value.count,
					versions: value.versions.join( ' ' ),
					message: { message: message, link: link }
				} );
			}

			newData.sort( ( a, b ) => {
				if ( a.count < b.count ) {
					return 1;
				} else if ( a.count > b.count ) {
					return -1;
				}
				return 0;
			} );
			items.value = newData;
		};

		onMounted( () => {
			intervalTimer = window.setInterval( populateLogs, INTERVAL );
			populateLogs();
		} );

		onUnmounted( () => {
			if ( intervalTimer ) {
				clearInterval( intervalTimer );
				intervalTimer = null;
			}
		} );

		return {
			headers,
			items,
			search,
			cdxIconLinkExternal
		};
	}
} );
</script>

<style lang="less">
@import ( reference ) '@wikimedia/codex-design-tokens/theme-wikimedia-ui.less';
@import ( reference ) '@wikimedia/codex/mixins/link.less';

.cdx-link {
	.cdx-mixin-link();

	.cdx-icon {
		color: inherit;
	}
}

.cdx-table {
	background-color: white;
}
</style>
