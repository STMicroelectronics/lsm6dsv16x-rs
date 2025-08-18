use st_mems_reg_config_conv::parser;
use std::path::Path;

fn main() {
    let input_file = Path::new("lsm6dsv16x_four_d.ucf");
    let output_file = Path::new("src/config_fsm.rs");
    parser::generate_rs_from_ucf(&input_file, &output_file, "FOUR_D");

    println!("cargo:rustc-link-arg-bins=--nmagic");
    println!("cargo:rustc-link-arg-bins=-Tlink.x");
    println!("cargo:rustc-link-arg-bins=-Tdefmt.x");
}
