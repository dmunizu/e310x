#[doc = "Register `rxctrl` reader"]
pub type R = crate::R<RxctrlSpec>;
#[doc = "Register `rxctrl` writer"]
pub type W = crate::W<RxctrlSpec>;
#[doc = "Field `enable` reader - "]
pub type EnableR = crate::BitReader;
#[doc = "Field `enable` writer - "]
pub type EnableW<'a, REG> = crate::BitWriter<'a, REG>;
#[doc = "Field `counter` reader - "]
pub type CounterR = crate::FieldReader;
#[doc = "Field `counter` writer - "]
pub type CounterW<'a, REG> = crate::FieldWriter<'a, REG, 3>;
impl R {
    #[doc = "Bit 0"]
    #[inline(always)]
    pub fn enable(&self) -> EnableR {
        EnableR::new((self.bits & 1) != 0)
    }
    #[doc = "Bits 16:18"]
    #[inline(always)]
    pub fn counter(&self) -> CounterR {
        CounterR::new(((self.bits >> 16) & 7) as u8)
    }
}
impl W {
    #[doc = "Bit 0"]
    #[inline(always)]
    pub fn enable(&mut self) -> EnableW<'_, RxctrlSpec> {
        EnableW::new(self, 0)
    }
    #[doc = "Bits 16:18"]
    #[inline(always)]
    pub fn counter(&mut self) -> CounterW<'_, RxctrlSpec> {
        CounterW::new(self, 16)
    }
}
#[doc = "Receive Control Register\n\nYou can [`read`](crate::Reg::read) this register and get [`rxctrl::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`rxctrl::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct RxctrlSpec;
impl crate::RegisterSpec for RxctrlSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`rxctrl::R`](R) reader structure"]
impl crate::Readable for RxctrlSpec {}
#[doc = "`write(|w| ..)` method takes [`rxctrl::W`](W) writer structure"]
impl crate::Writable for RxctrlSpec {
    type Safety = crate::Unsafe;
}
#[doc = "`reset()` method sets rxctrl to value 0"]
impl crate::Resettable for RxctrlSpec {}
