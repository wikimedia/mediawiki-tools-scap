"""
    scap.phorge_conduit
    ~~~~~~~~~~
    Facade for Conduit, Phabricator/Phorge's REST API. Meant for bot interactions

"""

import json

import requests


class PhorgeConduit(object):
    def __init__(self, phorge_url, bot_token, proxy=None):
        self.phorge_url = phorge_url
        self.bot_token = bot_token
        self.proxies = {"http": proxy, "https": proxy} if proxy else None

    def request(self, op, data):
        data["__conduit__"] = {"token": self.bot_token}
        res = requests.post(
            f"{self.phorge_url}/api/{op}",
            data={"params": json.dumps(data), "output": "json"},
            proxies=self.proxies,
        )
        json_res = res.json()
        if json_res["error_code"]:
            if self.bot_token in json_res["error_info"]:
                # When there is a problem with the token, Conduit returns the token in the error
                raise Exception("There is a problem with the supplied API token")
            else:
                raise Exception(json_res["error_info"])
        return json_res["result"]

    def data_request(self, op, data):
        return self.request(op, data)["data"]

    def user_by_name(self, username, constraints=None):
        all_constraints = {"usernames": [username]}
        if constraints:
            all_constraints.update(constraints)
        user_list = self.data_request("user.search", {"constraints": all_constraints})
        if len(user_list) == 0:
            return None
        return user_list[0]

    def base_task_info(self, task_id):
        task_id_int = int(task_id[1:])
        task_list = self.data_request(
            "maniphest.search", {"constraints": {"ids": [task_id_int]}}
        )
        if len(task_list) == 0:
            return None
        return task_list[0]

    def parents_of(self, task_id):
        task_phid = self.base_task_info(task_id)["phid"]
        parent_edges = self.data_request(
            "edge.search", {"sourcePHIDs": [task_phid], "types": ["task.parent"]}
        )
        if len(parent_edges) == 0:
            return []

        parents_phids = [edge["destinationPHID"] for edge in parent_edges]
        parents = self.data_request(
            "maniphest.search", {"constraints": {"phids": parents_phids}}
        )
        return [f"T{parent['id']}" for parent in parents]

    def task_comments(self, task_id, constraints=None):
        data = {"objectIdentifier": task_id}
        if constraints:
            data["constraints"] = constraints
        transactions = self.data_request("transaction.search", data)
        if len(transactions) == 0:
            return []
        return [
            transaction
            for transaction in transactions
            if transaction["type"] == "comment"
        ]

    def edit_task(self, task_id_str, transactions):
        data = {"objectIdentifier": task_id_str, "transactions": transactions}
        self.request("maniphest.edit", data)
