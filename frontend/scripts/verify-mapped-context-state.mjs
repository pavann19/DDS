import assert from "node:assert/strict";
import { readFileSync } from "node:fs";
import { mergeMappedContext } from "../src/lib/mappedContextState.ts";
const old={status:"available",recorded_fallback:true,recorded_asset_sha256:"a".repeat(64),features:[{osm_id:1}],infrastructure:[{osm_id:2}],refresh_error:"HTTPStatusError"};
const retry={...old,features:[{osm_id:1}],infrastructure:[{osm_id:2}],refresh_error:"ReadTimeout"};
const same=mergeMappedContext(old,retry);
assert.equal(same.features,old.features);assert.equal(same.infrastructure,old.infrastructure);assert.equal(same.refresh_error,"ReadTimeout");
const moved={...retry,recorded_asset_sha256:"b".repeat(64)};
assert.equal(mergeMappedContext(old,moved).features,moved.features);
const live={...retry,recorded_fallback:false};assert.equal(mergeMappedContext(old,live).features,live.features);
const invalid={...retry,recorded_asset_sha256:"untrusted"};assert.equal(mergeMappedContext(old,invalid).status,"unavailable_cached");assert.equal(mergeMappedContext(null,invalid).features.length,0);
assert.equal(mergeMappedContext(old,{...retry,status:"unavailable"}).status,"unavailable_cached");
console.log("Identical recorded map hashes retain geometry identity; new/live records replace; invalid results never become available.");

const source=readFileSync(new URL("../src/components/3d/MappedEnvironment.tsx",import.meta.url),"utf8");
assert(source.includes("setData(previous => mergeMappedContext(previous,next))"));
assert(source.includes("[mappedFeatures, mappedInfrastructure, route]"));
