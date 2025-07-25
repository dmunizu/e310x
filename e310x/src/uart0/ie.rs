#[doc = "Register `ie` reader"]
pub type R = crate::R<IeSpec>;
#[doc = "Register `ie` writer"]
pub type W = crate::W<IeSpec>;
#[doc = "Field `txwm` reader - "]
pub type TxwmR = crate::BitReader;
#[doc = "Field `txwm` writer - "]
pub type TxwmW<'a, REG> = crate::BitWriter<'a, REG>;
#[doc = "Field `rxwm` reader - "]
pub type RxwmR = crate::BitReader;
#[doc = "Field `rxwm` writer - "]
pub type RxwmW<'a, REG> = crate::BitWriter<'a, REG>;
impl R {
    #[doc = "Bit 0"]
    #[inline(always)]
    pub fn txwm(&self) -> TxwmR {
        TxwmR::new((self.bits & 1) != 0)
    }
    #[doc = "Bit 1"]
    #[inline(always)]
    pub fn rxwm(&self) -> RxwmR {
        RxwmR::new(((self.bits >> 1) & 1) != 0)
    }
}
impl W {
    #[doc = "Bit 0"]
    #[inline(always)]
    pub fn txwm(&mut self) -> TxwmW<'_, IeSpec> {
        TxwmW::new(self, 0)
    }
    #[doc = "Bit 1"]
    #[inline(always)]
    pub fn rxwm(&mut self) -> RxwmW<'_, IeSpec> {
        RxwmW::new(self, 1)
    }
}
#[doc = "Interrupt Enable Register\n\nYou can [`read`](crate::Reg::read) this register and get [`ie::R`](R). You can [`reset`](crate::Reg::reset), [`write`](crate::Reg::write), [`write_with_zero`](crate::Reg::write_with_zero) this register using [`ie::W`](W). You can also [`modify`](crate::Reg::modify) this register. See [API](https://docs.rs/svd2rust/#read--modify--write-api)."]
pub struct IeSpec;
impl crate::RegisterSpec for IeSpec {
    type Ux = u32;
}
#[doc = "`read()` method returns [`ie::R`](R) reader structure"]
impl crate::Readable for IeSpec {}
#[doc = "`write(|w| ..)` method takes [`ie::W`](W) writer structure"]
impl crate::Writable for IeSpec {
    type Safety = crate::Unsafe;
}
#[doc = "`reset()` method sets ie to value 0"]
impl crate::Resettable for IeSpec {}
