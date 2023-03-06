#[doc = r"Register block"]
#[repr(C)]
pub struct RegisterBlock {
    #[doc = "0x00 - Clock Configuration Register"]
    pub hfrosccfg: HFROSCCFG,
    #[doc = "0x04 - Clock Configuration Register"]
    pub hfxosccfg: HFXOSCCFG,
    #[doc = "0x08 - PLL Configuration Register"]
    pub pllcfg: PLLCFG,
    #[doc = "0x0c - PLL Divider Register"]
    pub plloutdiv: PLLOUTDIV,
    #[doc = "0x10 - Clock Configuration Register"]
    pub coreclkcfg: CORECLKCFG,
}
#[doc = "hfrosccfg (rw) register accessor: an alias for `Reg<HFROSCCFG_SPEC>`"]
pub type HFROSCCFG = crate::Reg<hfrosccfg::HFROSCCFG_SPEC>;
#[doc = "Clock Configuration Register"]
pub mod hfrosccfg;
#[doc = "hfxosccfg (rw) register accessor: an alias for `Reg<HFXOSCCFG_SPEC>`"]
pub type HFXOSCCFG = crate::Reg<hfxosccfg::HFXOSCCFG_SPEC>;
#[doc = "Clock Configuration Register"]
pub mod hfxosccfg;
#[doc = "pllcfg (rw) register accessor: an alias for `Reg<PLLCFG_SPEC>`"]
pub type PLLCFG = crate::Reg<pllcfg::PLLCFG_SPEC>;
#[doc = "PLL Configuration Register"]
pub mod pllcfg;
#[doc = "plloutdiv (rw) register accessor: an alias for `Reg<PLLOUTDIV_SPEC>`"]
pub type PLLOUTDIV = crate::Reg<plloutdiv::PLLOUTDIV_SPEC>;
#[doc = "PLL Divider Register"]
pub mod plloutdiv;
#[doc = "coreclkcfg (rw) register accessor: an alias for `Reg<CORECLKCFG_SPEC>`"]
pub type CORECLKCFG = crate::Reg<coreclkcfg::CORECLKCFG_SPEC>;
#[doc = "Clock Configuration Register"]
pub mod coreclkcfg;
