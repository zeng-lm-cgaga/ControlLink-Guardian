#!/usr/bin/env python3
"""P0-F1/BF1 声明式契约向量；不执行生产 session 引擎，不证明网络准入。"""

import copy
import json
from pathlib import Path
import sys
import unittest

import yaml


ROOT = Path(__file__).resolve().parents[2]
SCOPE = "declarative_contract_vectors_only_no_production_session_engine_or_network"
TOKEN_A = "11" * 16
TOKEN_B = "22" * 16
GID_A = "33" * 24
GID_B = "44" * 24
KEY_A = ("55" * 16, "66" * 16)
KEY_B = ("77" * 16, "88" * 16)
DOMAIN_A = {"service_instance_id": "aa" * 16, "domain_version": 1, "generation": 1}
DOMAIN_B = {"service_instance_id": "aa" * 16, "domain_version": 1, "generation": 2}
DOMAIN_OTHER_INSTANCE = {"service_instance_id": "bb" * 16, "domain_version": 1,
                         "generation": 2}
RELEASE_ORDER = ["confirm_revoked_epoch", "release_resources", "advance_opening_domain",
                 "reclaim_old_domain_keys"]
BUSINESS_FIELDS = ("business_sequence", "business_source_stamp_ns",
                   "business_receive_steady_ns", "business_lease_deadline_steady_ns")
READONLY_STATUS = {
    "publisher_instance_token": "99" * 16,
    "session": {"control_session_state": "ADMITTED", "ingress_publisher": {"gid": GID_A}},
    "latest_forwarded": {"key": {"source_sequence": 7}},
    "latest_gateway_acceptance": {"key": {"source_sequence": 6}},
}


def holder(transport, peer, key, domain=DOMAIN_A):
    return {"transport": transport,
            "get_client" if transport == "someip" else "local_rpc_id": peer,
            "holder_nonce": key[0], "open_request_id": key[1],
            "opening_domain": copy.deepcopy(domain)}


def declared_snapshot(state, epoch, identity=None, token="", gid="", ready=False,
                      sequence=None, pending=None, retired=(), domain=DOMAIN_A):
    return {
        "state": state, "epoch": epoch, "holder": identity,
        "session_token": token, "gid": gid, "ready": ready,
        "business_sequence": sequence, "business_source_stamp_ns": 1000000,
        "business_receive_steady_ns": 2000000,
        "business_lease_deadline_steady_ns": 3000000,
        "network_idle_deadline_steady_ns": 9000000,
        "graph_deadline_steady_ns": 8000000,
        "control_absolute_deadline_steady_ns": 600000000000,
        "pending": pending, "retired_keys": [list(key) for key in retired],
        "opening_domain": copy.deepcopy(domain),
    }


def declared_vectors(transport):
    """装配预期事实，不推进状态；满容量是当前域的合成边界，不是实际运行轨迹。"""
    peer_a, peer_b = (0x31, 0x32) if transport == "someip" else ("rpc-a", "rpc-b")
    identity_a = holder(transport, peer_a, KEY_A)
    competitor_b = holder(transport, peer_b, KEY_B)
    identity_b = holder(transport, peer_b, KEY_B, DOMAIN_B)
    idle = declared_snapshot("IDLE", 0)
    opening = declared_snapshot("OPENING", 1, identity_a, TOKEN_A, GID_A)
    admitted = declared_snapshot("ADMITTED", 1, identity_a, TOKEN_A, GID_A, True)
    working = declared_snapshot("ADMITTED", 1, identity_a, TOKEN_A, GID_A, True, 7,
                                {"epoch": 1, "sequence": 8})
    revoking = declared_snapshot("REVOKING", 2, identity_a, TOKEN_A, GID_A, False, 7)
    released = declared_snapshot("IDLE", 2, domain=DOMAIN_B)
    opening_b = declared_snapshot("OPENING", 3, identity_b, TOKEN_B, GID_B, domain=DOMAIN_B)
    admitted_b = declared_snapshot("ADMITTED", 3, identity_b, TOKEN_B, GID_B, True,
                                   domain=DOMAIN_B)
    capacity = yaml.safe_load((ROOT / "config/adas/protocol_transports.yaml").read_text())["control_session_resources"]["retired_open_capacity"]
    retired_full = (KEY_A,) + tuple((f"{index:032x}", f"{index + 100:032x}")
                                   for index in range(capacity - 1))
    full_idle = declared_snapshot("IDLE", 2, retired=retired_full)
    full_working = declared_snapshot("ADMITTED", 1, identity_a, TOKEN_A, GID_A, True, 7,
                                     {"epoch": 1, "sequence": 8}, retired_full)
    full_revoking = declared_snapshot("REVOKING", 2, identity_a, TOKEN_A, GID_A,
                                      False, 7, retired=retired_full)
    reused_identity = holder(transport, peer_a, KEY_A, DOMAIN_B)
    reopened_same_keys = declared_snapshot("OPENING", 3, reused_identity, TOKEN_B, GID_B,
                                           domain=DOMAIN_B)
    missing_domain_request = copy.deepcopy(identity_a)
    del missing_domain_request["opening_domain"]

    def vector(name, event, before, after, request=None, result=None, response_session=None,
               forwarded=False, release_facts=None):
        return {"name": transport + "/" + name, "transport": transport,
                "event": event, "before": before, "after": after,
                "request": request, "result": result,
                "response_session": response_session, "forwarded": forwarded,
                "release_facts": release_facts}

    release_facts = {"revoked_epoch": 2, "resources_released": True,
                     "order": RELEASE_ORDER}

    vectors = [
        vector("first-open", "first_open", idle, opening, identity_a, "OPENING", opening),
        vector("graph-admitted", "graph_admitted", opening, admitted, result="ADMITTED",
               response_session=admitted),
        vector("lost-opening-response-retry", "same_holder_retry", opening, opening,
               identity_a, "IDEMPOTENT", opening),
        vector("lost-admitted-response-retry", "same_holder_retry", working, working,
               identity_a, "IDEMPOTENT", working),
        vector("second-peer-same-key", "competing_open", working, working,
               holder(transport, peer_b, KEY_A), "BUSY"),
        vector("second-peer-new-key", "competing_open", opening, opening,
               competitor_b, "BUSY"),
        vector("same-peer-changed-key", "competing_open", working, working,
               holder(transport, peer_a, KEY_B), "BUSY"),
        vector("readonly-cancel", "readonly_cancel", working, working),
        vector("readonly-reconnect", "readonly_reconnect", opening, opening),
        vector("readonly-query", "readonly_query", working, working),
        vector("transparent-connectivity-hint", "transport_hint", working, working),
        vector("explicit-switch-close", "explicit_close", working, revoking, identity_a,
               "REVOKED"),
        vector("explicit-switch-release", "release_complete", revoking, released,
               release_facts=release_facts),
        vector("explicit-switch-new-open", "new_holder_open", released, opening_b,
               identity_b, "OPENING", opening_b),
        vector("explicit-switch-new-admission", "graph_admitted", opening_b, admitted_b,
               result="ADMITTED", response_session=admitted_b),
        vector("revoking-open-busy", "revoking_open", revoking, revoking, competitor_b, "BUSY"),
        vector("retired-key-cannot-revive", "retired_key_open", full_idle, full_idle,
               holder(transport, peer_b, KEY_A), "REJECTED"),
        vector("current-domain-capacity-full-idle", "capacity_full_open", full_idle, full_idle,
               competitor_b, "RESOURCE_EXHAUSTED"),
        vector("current-domain-capacity-full-held", "capacity_full_open", full_working, full_working,
               competitor_b, "BUSY"),
        vector("current-domain-capacity-full-idempotent", "same_holder_retry", full_working, full_working,
               identity_a, "IDEMPOTENT", full_working),
        vector("current-domain-full-release", "release_complete", full_revoking, released,
               release_facts=release_facts),
        vector("old-domain-replay-after-reclamation", "old_domain_open", released, released,
               identity_a, "REJECTED"),
        vector("new-explicit-domain-same-keys", "new_holder_open", released, reopened_same_keys,
               reused_identity, "OPENING", reopened_same_keys),
        vector("old-domain-before-same-holder-match", "old_domain_open", reopened_same_keys,
               reopened_same_keys, identity_a, "REJECTED"),
        vector("other-service-instance-replay", "old_domain_open", released, released,
               holder(transport, peer_a, KEY_A, DOMAIN_OTHER_INSTANCE), "REJECTED"),
        vector("missing-domain-schema-one-open", "missing_domain_open", idle, idle,
               dict(missing_domain_request, schema_version=1), "REJECTED"),
        vector("old-token-new-timestamp", "old_token_submit", admitted_b, admitted_b,
               {"session_token": TOKEN_A, "source_sequence": 8,
                "source_stamp_ns": 9999999}, "REJECTED"),
        vector("old-completion-after-switch", "old_epoch_completion", admitted_b, admitted_b,
               {"epoch": 1, "session_token": TOKEN_A, "source_sequence": 8}, "REJECTED"),
        vector("old-completion-during-revoke", "old_epoch_completion", revoking, revoking,
               {"epoch": 1, "session_token": TOKEN_A, "source_sequence": 8}, "REJECTED"),
        vector("other-peer-close-rejected", "competing_close", working, working,
               holder(transport, peer_b, KEY_A), "REJECTED"),
    ]
    failures = ["protocol_failure", "graph_timeout", "stop", "random_source_failure"]
    failures += (["someip_idle_expired", "application_deregistered", "service_withdrawn"]
                 if transport == "someip" else ["read_eof", "done", "absolute_deadline"])
    for event in failures:
        vectors.append(vector(event, event, working, revoking, result="REVOKED"))
        opening_revoked = declared_snapshot("REVOKING", 2, identity_a, TOKEN_A, GID_A)
        vectors.append(vector("opening-" + event, event, opening, opening_revoked, result="REVOKED"))
    if transport == "someip":
        kept = copy.deepcopy(opening)
        kept["network_idle_deadline_steady_ns"] = 10000000
        request = dict(identity_a, keepalive_sequence=0)
        vectors.append(vector("opening-valid-keep", "someip_keep", opening, kept, request))
        vectors.append(vector("duplicate-keep-no-renewal", "duplicate_keep", kept, kept,
                              request))
    else:
        vectors.append(vector("keep-does-not-move-rpc-deadline", "grpc_keep", working, working,
                              dict(identity_a, keepalive_sequence=0)))
    return vectors


VECTORS = declared_vectors("someip") + declared_vectors("grpc")
TERMINATIONS = {"explicit_close", "protocol_failure", "graph_timeout", "stop",
                "random_source_failure", "someip_idle_expired", "application_deregistered",
                "service_withdrawn", "read_eof", "done", "absolute_deadline"}


def require(condition, description):
    if not condition:
        raise ValueError(description)


def check_readonly_tokens(value):
    if isinstance(value, dict):
        for name, member in value.items():
            require(name != "session_token" or member == "", "readonly control token disclosure")
            check_readonly_tokens(member)
    elif isinstance(value, list):
        for member in value:
            check_readonly_tokens(member)


def check_vector(vector):
    """核对声明的契约关系；不返回或提交新 session 状态。"""
    before, after = vector["before"], vector["after"]
    event, request = vector["event"], vector["request"]
    for snapshot in (before, after):
        require(snapshot["state"] in {"IDLE", "OPENING", "ADMITTED", "REVOKING"}, "unknown state")
        require(snapshot["ready"] is (snapshot["state"] == "ADMITTED"), "readiness inconsistent")
        domain = snapshot["opening_domain"]
        require(domain["domain_version"] == 1 and
                len(bytes.fromhex(domain["service_instance_id"])) == 16 and
                0 < domain["generation"] <= (1 << 64) - 1, "invalid declared opening domain")
        if snapshot["holder"] is not None:
            identity = snapshot["holder"]
            peer_field = "get_client" if vector["transport"] == "someip" else "local_rpc_id"
            require(set(identity) == {"transport", peer_field, "holder_nonce", "open_request_id", "opening_domain"}, "holder identity incomplete")
            require(identity["transport"] == vector["transport"], "holder transport mismatch")
            require(identity["opening_domain"] == domain, "holder escaped opening domain")
            require(all(len(bytes.fromhex(identity[field])) == 16 for field in
                        ("holder_nonce", "open_request_id")), "invalid holder/key width")
            require(len(bytes.fromhex(snapshot["session_token"])) == 16, "invalid token width")
            require(len(bytes.fromhex(snapshot["gid"])) == 24, "invalid GID width")
        else:
            require(snapshot["state"] == "IDLE" and not snapshot["session_token"] and not snapshot["gid"], "unowned session disclosure")
    response = vector["response_session"]
    if vector["result"] in {"BUSY", "REJECTED", "RESOURCE_EXHAUSTED"}:
        require(response is None, "competitor must not receive session/token")
    require(not vector["forwarded"], "network lifecycle vector cannot forward a business command")
    require(all(before[field] == after[field] for field in BUSINESS_FIELDS[1:]), "network event refreshed business time/lease")
    if event != "release_complete":
        require(before["opening_domain"] == after["opening_domain"], "non-release event advanced opening domain")
    if request is not None and "opening_domain" in request and event != "old_domain_open":
        require(request["opening_domain"] == before["opening_domain"], "request must explicitly bind current opening domain")
    if event in {"first_open", "new_holder_open"}:
        require(before["state"] == "IDLE" and after["state"] == "OPENING", "invalid open transition")
        require(after["holder"] == request and after["epoch"] > before["epoch"], "new holder must own fresh epoch")
        require([request["holder_nonce"], request["open_request_id"]] not in before["retired_keys"], "retired key reopened")
        require(vector["result"] == "OPENING" and response == after, "open response inconsistent")
    elif event == "graph_admitted":
        require(before["state"] == "OPENING" and after["state"] == "ADMITTED", "invalid admission transition")
        require(vector["result"] == "ADMITTED" and response == after, "admission response inconsistent")
        require({field: value for field, value in before.items() if field not in {"state", "ready"}} ==
                {field: value for field, value in after.items() if field not in {"state", "ready"}}, "graph admission changed generation or business facts")
    elif event == "same_holder_retry":
        require(request == before["holder"] and before == after, "idempotent retry rotated identity/time")
        require(vector["result"] == "IDEMPOTENT" and response == before, "idempotent response inconsistent")
    elif event in {"competing_open", "competing_close"}:
        require(request != before["holder"] and before == after, "competitor stole holder")
        require(vector["result"] == ("BUSY" if event == "competing_open" else "REJECTED"), "competitor result inconsistent")
    elif event in TERMINATIONS:
        require(before["state"] in {"OPENING", "ADMITTED"} and after["state"] == "REVOKING", "termination did not revoke")
        require(after["epoch"] > before["epoch"] and after["pending"] is None and not after["ready"], "termination retained old pending epoch")
        require(after["holder"] == before["holder"] and after["business_sequence"] == before["business_sequence"], "termination reset business generation before release")
        require(vector["result"] == "REVOKED", "termination result inconsistent")
        if event == "explicit_close":
            require(request == before["holder"], "foreign close revoked holder")
    elif event == "release_complete":
        require(before["state"] == "REVOKING" and after["state"] == "IDLE", "release skipped revoke")
        require(after["epoch"] == before["epoch"] and after["pending"] is None, "release revived old epoch")
        facts = vector["release_facts"]
        require(facts is not None and facts["revoked_epoch"] == before["epoch"] and
                facts["resources_released"] is True and facts["order"] == RELEASE_ORDER,
                "reclamation requires released resources and domain advancement first")
        require(after["opening_domain"] == dict(before["opening_domain"],
                generation=before["opening_domain"]["generation"] + 1), "release must advance non-reusable opening domain")
        require(not after["retired_keys"], "released domain keys still consume current-domain budget")
    elif event == "someip_keep":
        require(request["holder_nonce"] == before["holder"]["holder_nonce"] and
                request["open_request_id"] == before["holder"]["open_request_id"], "keep holder mismatch")
        require(after["network_idle_deadline_steady_ns"] > before["network_idle_deadline_steady_ns"], "valid keep did not renew network idle")
        require({field: value for field, value in before.items() if field != "network_idle_deadline_steady_ns"} ==
                {field: value for field, value in after.items() if field != "network_idle_deadline_steady_ns"}, "keep changed business/Graph facts")
    else:
        require(before == after, "rejected/readonly/transparent event changed holder")
        if event == "retired_key_open":
            require([request["holder_nonce"], request["open_request_id"]] in before["retired_keys"] and
                    vector["result"] == "REJECTED", "retired key revived")
        elif event == "capacity_full_open":
            capacity = yaml.safe_load((ROOT / "config/adas/protocol_transports.yaml").read_text())["control_session_resources"]["retired_open_capacity"]
            require(len(before["retired_keys"]) == capacity and
                    vector["result"] == ("RESOURCE_EXHAUSTED" if before["state"] == "IDLE" else "BUSY"),
                    "current-domain capacity exhaustion evicted live-domain key")
        elif event == "old_domain_open":
            require(request["opening_domain"] != before["opening_domain"] and
                    vector["result"] == "REJECTED", "old domain revived even if holder keys match")
        elif event == "missing_domain_open":
            require("opening_domain" not in request and request["schema_version"] == 1 and
                    vector["result"] == "REJECTED", "schema version one inferred missing domain")
        elif event == "old_token_submit":
            require(request["session_token"] != before["session_token"] and
                    request["source_stamp_ns"] > before["business_source_stamp_ns"] and
                    vector["result"] == "REJECTED", "new timestamp bypassed old token rejection")
        elif event == "old_epoch_completion":
            require(request["epoch"] != before["epoch"] and vector["result"] == "REJECTED", "old completion crossed epoch")
        elif event == "revoking_open":
            require(before["state"] == "REVOKING" and vector["result"] == "BUSY", "new holder entered cleanup gap")
        else:
            require(event in {"readonly_cancel", "readonly_reconnect", "readonly_query", "transport_hint",
                              "duplicate_keep", "grpc_keep"}, "unknown vector event")


class SessionContractVectorsTest(unittest.TestCase):
    def test_all_declarative_vectors(self):
        for vector in VECTORS:
            with self.subTest(vector=vector["name"]):
                check_vector(vector)

    def test_unique_cases_and_deterministic_encoding(self):
        self.assertEqual(len(VECTORS), len({vector["name"] for vector in VECTORS}))
        self.assertEqual(json.dumps(VECTORS, sort_keys=True),
                         json.dumps(declared_vectors("someip") + declared_vectors("grpc"), sort_keys=True))

    def test_exported_json_vectors_replay(self):
        restored = json.loads(json.dumps(VECTORS, sort_keys=True))
        self.assertEqual(restored, VECTORS)
        for vector in restored:
            with self.subTest(vector=vector["name"]):
                check_vector(vector)

    def test_explicit_switch_sequence_has_fresh_key_token_gid_epoch(self):
        for transport in ("someip", "grpc"):
            path = [vector for vector in VECTORS if vector["transport"] == transport and
                    vector["name"].split("/")[1].startswith("explicit-switch-")]
            self.assertEqual([vector["event"] for vector in path],
                             ["explicit_close", "release_complete", "new_holder_open", "graph_admitted"])
            for previous, current in zip(path, path[1:]):
                self.assertEqual(previous["after"], current["before"])
            before, after = path[0]["before"], path[-1]["after"]
            for field in ("epoch", "session_token", "gid", "holder", "opening_domain"):
                self.assertNotEqual(before[field], after[field])

    def test_second_rpc_same_nonce_key_is_still_busy(self):
        vector = next(vector for vector in VECTORS if vector["name"] == "grpc/second-peer-same-key")
        self.assertNotEqual(vector["request"]["local_rpc_id"], vector["before"]["holder"]["local_rpc_id"])
        for field in ("holder_nonce", "open_request_id"):
            self.assertEqual(vector["request"][field], vector["before"]["holder"][field])
        check_vector(vector)

    def test_same_keys_need_new_explicit_domain_not_rewritten_old_request(self):
        for transport in ("someip", "grpc"):
            fresh = next(vector for vector in VECTORS
                         if vector["name"] == transport + "/new-explicit-domain-same-keys")
            replay = next(vector for vector in VECTORS
                          if vector["name"] == transport + "/old-domain-before-same-holder-match")
            for field in ("holder_nonce", "open_request_id"):
                self.assertEqual(fresh["request"][field], replay["request"][field])
            self.assertNotEqual(fresh["request"]["opening_domain"], replay["request"]["opening_domain"])
            self.assertEqual(fresh["result"], "OPENING")
            self.assertEqual(replay["result"], "REJECTED")
            check_vector(fresh)
            check_vector(replay)

    def test_readonly_fixture_preserves_public_identity_without_control_tokens(self):
        check_readonly_tokens(READONLY_STATUS)
        self.assertEqual(READONLY_STATUS["publisher_instance_token"], "99" * 16)
        self.assertEqual(READONLY_STATUS["session"]["ingress_publisher"]["gid"], GID_A)
        self.assertEqual(READONLY_STATUS["latest_forwarded"]["key"]["source_sequence"], 7)
        for path in (("session",), ("latest_forwarded", "key"),
                     ("latest_gateway_acceptance", "key")):
            fixture = copy.deepcopy(READONLY_STATUS)
            destination = fixture
            for member in path:
                destination = destination[member]
            destination["session_token"] = TOKEN_A
            with self.subTest(path=path), self.assertRaisesRegex(ValueError, "readonly control token"):
                check_readonly_tokens(fixture)

    def test_negative_retry_token_rotation_and_time_refresh(self):
        original = next(vector for vector in VECTORS if vector["name"] == "grpc/lost-admitted-response-retry")
        for field, value in (("session_token", TOKEN_B), ("gid", GID_B), ("epoch", 99),
                             ("business_source_stamp_ns", 9999999), ("business_sequence", 0),
                             ("network_idle_deadline_steady_ns", 10000000),
                             ("graph_deadline_steady_ns", 10000000),
                             ("control_absolute_deadline_steady_ns", 700000000000),
                             ("opening_domain", DOMAIN_B)):
            vector = copy.deepcopy(original)
            vector["after"] = copy.deepcopy(original["after"])
            vector["after"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_vector(vector)

    def test_negative_busy_token_disclosure_or_holder_takeover(self):
        original = next(vector for vector in VECTORS if vector["name"] == "someip/second-peer-same-key")
        vector = copy.deepcopy(original)
        vector["response_session"] = vector["before"]
        with self.assertRaisesRegex(ValueError, "competitor must not receive"):
            check_vector(vector)
        vector = copy.deepcopy(original)
        vector["after"] = copy.deepcopy(original["after"])
        vector["after"]["holder"] = vector["request"]
        with self.assertRaisesRegex(ValueError, "competitor stole"):
            check_vector(vector)

    def test_negative_terminal_epoch_or_pending_not_revoked(self):
        original = next(vector for vector in VECTORS if vector["name"] == "grpc/done")
        for field, value in (("epoch", 1), ("pending", {"epoch": 1, "sequence": 8})):
            vector = copy.deepcopy(original)
            vector["after"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_vector(vector)

    def test_negative_old_completion_or_old_token_forwarded(self):
        for name in ("grpc/old-completion-after-switch", "someip/old-token-new-timestamp"):
            vector = copy.deepcopy(next(vector for vector in VECTORS if vector["name"] == name))
            vector["forwarded"] = True
            with self.subTest(vector=name), self.assertRaises(ValueError):
                check_vector(vector)

    def test_negative_live_domain_keys_evicted_at_capacity(self):
        vector = copy.deepcopy(next(vector for vector in VECTORS if vector["name"] == "someip/current-domain-capacity-full-idle"))
        vector["after"] = copy.deepcopy(vector["after"])
        vector["after"]["retired_keys"] = vector["after"]["retired_keys"][1:]
        with self.assertRaises(ValueError):
            check_vector(vector)

    def test_negative_reclamation_before_new_domain_or_resource_release(self):
        original = next(vector for vector in VECTORS
                        if vector["name"] == "grpc/current-domain-full-release")
        for field, value in (("revoked_epoch", 1), ("resources_released", False),
                             ("order", ["confirm_revoked_epoch", "release_resources",
                                        "reclaim_old_domain_keys", "advance_opening_domain"])):
            vector = copy.deepcopy(original)
            vector["release_facts"][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                check_vector(vector)
        vector = copy.deepcopy(original)
        vector["after"]["opening_domain"] = copy.deepcopy(vector["before"]["opening_domain"])
        with self.assertRaises(ValueError):
            check_vector(vector)


if __name__ == "__main__":
    if "--dump-vectors" in sys.argv:
        print(json.dumps({"scope": SCOPE, "vectors": VECTORS,
                          "readonly_status": READONLY_STATUS}, sort_keys=True, indent=2))
    else:
        unittest.main()
