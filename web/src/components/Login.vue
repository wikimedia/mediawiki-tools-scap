<template>
    <cdx-field :status="status" :messages="messages">

        <template #label>Please Login</template>
        <template #description>
            <p>Login in with the username that the <code>spiderpig-apiserver</code> process is running under.</p>
        </template>
        <template #help-text>
            <p>Run <code>scap spiderpig-otp</code> to generate a password.</p>
        </template>

        <cdx-label input-id="username">Username</cdx-label>
        <cdx-text-input id="username" v-model="username" />
        <cdx-label input-id="password">One time password</cdx-label>
        <cdx-text-input id="password" v-model="password" />
    </cdx-field>

    <cdx-button :disabled="loginButtonDisabled" @click="login">Login</cdx-button>
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