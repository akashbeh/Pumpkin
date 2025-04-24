use async_trait::async_trait;
use pumpkin_data::block::Block;
use pumpkin_data::block::BlockProperties;
use pumpkin_data::block::BlockState;
use pumpkin_data::block::DoorHinge;
use pumpkin_data::block::DoubleBlockHalf;
use pumpkin_data::tag::RegistryKey;
use pumpkin_data::tag::get_tag_values;
use pumpkin_util::GameMode;
use pumpkin_util::math::position::BlockPos;
use pumpkin_world::BlockStateId;
use pumpkin_world::block::BlockDirection;
use std::sync::Arc;

use crate::block::pumpkin_block::{BlockMetadata, PumpkinBlock};
use crate::block::registry::BlockActionResult;
use crate::entity::player::Player;
use crate::world::BlockFlags;
use pumpkin_data::item::Item;
use pumpkin_protocol::server::play::SUseItemOn;

use crate::server::Server;
use crate::world::World;

type DoorProperties = pumpkin_data::block::OakDoorLikeProperties;

async fn toggle_door(world: &Arc<World>, block_pos: &BlockPos) {
    let (block, block_state) = world.get_block_and_block_state(block_pos).await.unwrap();
    let mut door_props = DoorProperties::from_state_id(block_state.id, &block);
    door_props.open = door_props.open.flip();

    let other_half = match door_props.half {
        DoubleBlockHalf::Upper => BlockDirection::Down,
        DoubleBlockHalf::Lower => BlockDirection::Up,
    };
    let other_pos = block_pos.offset(other_half.to_offset());

    let (other_block, other_state_id) = world.get_block_and_block_state(&other_pos).await.unwrap();
    let mut other_door_props = DoorProperties::from_state_id(other_state_id.id, &other_block);
    other_door_props.open = door_props.open;

    world
        .set_block_state(
            block_pos,
            door_props.to_state_id(&block),
            BlockFlags::NOTIFY_LISTENERS,
        )
        .await;
    world
        .set_block_state(
            &other_pos,
            other_door_props.to_state_id(&other_block),
            BlockFlags::NOTIFY_LISTENERS,
        )
        .await;
}

fn can_open_door(block: &Block, player: &Player) -> bool {
    if block.id == Block::IRON_DOOR.id && player.gamemode.load() != GameMode::Creative {
        return false;
    }

    true
}

pub struct DoorBlock;
impl BlockMetadata for DoorBlock {
    fn namespace(&self) -> &'static str {
        "minecraft"
    }

    fn ids(&self) -> &'static [&'static str] {
        get_tag_values(RegistryKey::Block, "minecraft:doors").unwrap()
    }
}

#[async_trait]
impl PumpkinBlock for DoorBlock {
    async fn on_place(
        &self,
        _server: &Server,
        _world: &World,
        block: &Block,
        _face: &BlockDirection,
        _block_pos: &BlockPos,
        _use_item_on: &SUseItemOn,
        player: &Player,
        _other: bool,
    ) -> BlockStateId {
        let mut door_props = DoorProperties::default(block);
        door_props.half = DoubleBlockHalf::Lower;
        door_props.facing = player.living_entity.entity.get_horizontal_facing();
        door_props.hinge = DoorHinge::Left;

        door_props.to_state_id(block)
    }

    async fn can_place_at(&self, world: &World, block_pos: &BlockPos) -> bool {
        if world
            .get_block_state(&block_pos.offset(BlockDirection::Up.to_offset()))
            .await
            .is_ok_and(|state| state.replaceable)
        {
            return true;
        }
        false
    }

    async fn placed(
        &self,
        world: &Arc<World>,
        block: &Block,
        state_id: BlockStateId,
        block_pos: &BlockPos,
        _old_state_id: BlockStateId,
        _notify: bool,
    ) {
        let mut door_props = DoorProperties::from_state_id(state_id, block);
        door_props.half = DoubleBlockHalf::Upper;

        world
            .set_block_state(
                &block_pos.offset(BlockDirection::Up.to_offset()),
                door_props.to_state_id(block),
                BlockFlags::NOTIFY_ALL | BlockFlags::SKIP_BLOCK_ADDED_CALLBACK,
            )
            .await;
    }

    async fn broken(
        &self,
        block: &Block,
        _player: &Player,
        location: BlockPos,
        _server: &Server,
        world: Arc<World>,
        state: BlockState,
    ) {
        let door_props = DoorProperties::from_state_id(state.id, block);

        let other_half = match door_props.half {
            DoubleBlockHalf::Upper => BlockDirection::Down,
            DoubleBlockHalf::Lower => BlockDirection::Up,
        };

        let other_pos = location.offset(other_half.to_offset());

        if let Ok(other_block) = world.get_block(&other_pos).await {
            if other_block.id == block.id {
                world
                    .break_block(&other_pos, None, BlockFlags::NOTIFY_NEIGHBORS)
                    .await;
            }
        }
    }

    async fn use_with_item(
        &self,
        block: &Block,
        player: &Player,
        location: BlockPos,
        _item: &Item,
        _server: &Server,
        world: &Arc<World>,
    ) -> BlockActionResult {
        if !can_open_door(block, player) {
            return BlockActionResult::Continue;
        }

        toggle_door(world, &location).await;

        BlockActionResult::Consume
    }

    async fn normal_use(
        &self,
        block: &Block,
        player: &Player,
        location: BlockPos,
        _server: &Server,
        world: &Arc<World>,
    ) {
        if !can_open_door(block, player) {
            return;
        }

        toggle_door(world, &location).await;
    }
}
