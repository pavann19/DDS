type ContextShape={status:string;features:unknown[];infrastructure?:unknown[];recorded_fallback?:boolean;recorded_asset_sha256?:string};
/** Hash identity applies only to integrity-validated recorded assets; preserve
 * metadata updates while avoiding expensive geometry rebuilds each retry. */
export function mergeMappedContext<T extends ContextShape>(previous:T|null,next:T):T{
 const valid=next.status==="available"&&Array.isArray(next.features)&&(!next.infrastructure||Array.isArray(next.infrastructure))&&(!next.recorded_fallback||/^[a-f0-9]{64}$/.test(next.recorded_asset_sha256??""));
 if(!valid)return previous ? {...previous,status:"unavailable_cached"} : {...next,status:"unavailable",features:[],infrastructure:[]};
 if(previous?.recorded_fallback&&next.recorded_fallback&&previous.recorded_asset_sha256===next.recorded_asset_sha256)return {...next,features:previous.features,infrastructure:previous.infrastructure};
 return next;
}
