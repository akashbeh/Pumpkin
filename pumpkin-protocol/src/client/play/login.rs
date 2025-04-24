use pumpkin_data::packet::clientbound::PLAY_LOGIN;
use pumpkin_util::math::position::BlockPos;

use pumpkin_macros::packet;
use serde::Serialize;

use crate::{VarInt, codec::identifier::Identifier};

#[derive(Serialize)]
#[packet(PLAY_LOGIN)]
pub struct CLogin<'a> {
    entity_id: i32,
    is_hardcore: bool,
    dimension_names: &'a [Identifier],
    max_players: VarInt,
    view_distance: VarInt,
    simulated_distance: VarInt,
    reduced_debug_info: bool,
    enabled_respawn_screen: bool,
    limited_crafting: bool,
    // Spawn info
    dimension_type: VarInt,
    dimension_name: Identifier,
    /// First 8 bytes of the SHA-256 hash of the world's seed. Used client side for biome noise
    hashed_seed: i64,
    game_mode: u8,
    previous_gamemode: i8,
    debug: bool,
    is_flat: bool,
    death_dimension_name: Option<(Identifier, BlockPos)>,
    portal_cooldown: VarInt,
    sealevel: VarInt,
    enforce_secure_chat: bool,
}

impl<'a> CLogin<'a> {
    #[expect(clippy::too_many_arguments)]
    pub fn new(
        entity_id: i32,
        is_hardcore: bool,
        dimension_names: &'a [Identifier],
        max_players: VarInt,
        view_distance: VarInt,
        simulated_distance: VarInt,
        reduced_debug_info: bool,
        enabled_respawn_screen: bool,
        limited_crafting: bool,
        dimension_type: VarInt,
        dimension_name: Identifier,
        hashed_seed: i64,
        game_mode: u8,
        previous_gamemode: i8,
        debug: bool,
        is_flat: bool,
        death_dimension_name: Option<(Identifier, BlockPos)>,
        portal_cooldown: VarInt,
        sealevel: VarInt,
        enforce_secure_chat: bool,
    ) -> Self {
        Self {
            entity_id,
            is_hardcore,
            dimension_names,
            max_players,
            view_distance,
            simulated_distance,
            reduced_debug_info,
            enabled_respawn_screen,
            limited_crafting,
            dimension_type,
            dimension_name,
            hashed_seed,
            game_mode,
            previous_gamemode,
            debug,
            is_flat,
            death_dimension_name,
            portal_cooldown,
            sealevel,
            enforce_secure_chat,
        }
    }
}
