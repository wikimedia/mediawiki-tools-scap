<template>
    <form @submit.prevent="login">
        <cdx-field :status="status" :messages="messages">
            <template #label>
                Please Login
            </template>

            <template #description>
                Login in with the username that the
                <code>spiderpig-apiserver</code> process is running under.
            </template>

            <cdx-label input-id="username">Username</cdx-label>
            <cdx-text-input id="username" v-model="username" />

            <cdx-label input-id="password">
                One time password
                <template #description>Run <code>scap spiderpig-otp</code> to generate a password.</template>
            </cdx-label>
            <cdx-text-input id="password" input-type="password" v-model="password" />

            <br />

            <cdx-button action="progressive" weight="primary" :disabled="loginButtonDisabled" @click="login">
                Login
            </cdx-button>
        </cdx-field>
    </form>
</template>

<script>
import { CdxButton, CdxField, CdxTextInput, CdxLabel } from '@wikimedia/codex';

import useApi from '../api';

export default {
    components: {
        CdxButton,
        CdxField,
        CdxTextInput,
        CdxLabel
    },
    data() {
        return {
            api: null,
            status: "default",
            username: "",
            password: "",
            messages: {}
        }
    },
    mounted() {
        this.api = useApi()
    },
    computed: {
        loginButtonDisabled() {
            return (this.username.trim() == "" || this.password.trim() == "")
        }
    },
    methods: {
        async login() {
            const username = this.username.trim();
            const password = this.password.trim();

            const res = await this.api.login(username, password)

            if (res !== true) {
                this.status = "error"
                this.messages.error = res.statusText
                return
            }

            // Login was successful
            this.$router.push(this.$route.query.redirect || "/")
        },
        clearErrorStatus() {
            if (this.status === "error") {
                this.status = "default"
            }
        }
    },
    watch: {
        username() {
            this.clearErrorStatus()
        },
        password() {
            this.clearErrorStatus()
        },
    }
}
</script>

<style scoped></style>