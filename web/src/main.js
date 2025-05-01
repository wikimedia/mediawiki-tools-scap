import { createApp } from 'vue';
import { createPinia } from 'pinia';
import router from './router';
import { spLight } from './themes';
import App from './App.vue';
import './assets/reset.css';

// Vuetify
import '@mdi/font/css/materialdesignicons.css';
import 'vuetify/styles';
import '@mdi/font/css/materialdesignicons.css';
import { createVuetify } from 'vuetify';

const vuetify = createVuetify( {
	theme: {
		defaultTheme: 'spLight',
		themes: {
			spLight
		}
	},
	defaults: {
		VField: {
			bgColor: 'white'
		}
	}
} );

createApp( App )
	.use( router )
	.use( createPinia() )
	.use( vuetify )
	.mount( '#app' );
